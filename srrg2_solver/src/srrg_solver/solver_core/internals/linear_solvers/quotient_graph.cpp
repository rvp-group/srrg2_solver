#include "quotient_graph.h"
#include <algorithm>
#include <cassert>

namespace srrg2_solver {

  void setDifferenceInPlace(QuotientGraph::NodePtrSet& target,
                            const QuotientGraph::NodePtrSet& subtracted) {
    auto it_t = target.begin();
    auto it_s = subtracted.begin();
    while (it_t != target.end() && it_s != subtracted.end()) {
      if (*it_t == *it_s) {
        it_t = target.erase(it_t);
        ++it_s;
        continue;
      }
      if ((*it_t)->id() < (*it_s)->id())
        ++it_t;
      else
        ++it_s;
    }
  }

  int setDifferenceSum(const QuotientGraph::NodePtrSet& target,
                       const QuotientGraph::NodePtrSet& subtracted) {
    auto it_t = target.begin();
    auto it_s = subtracted.begin();
    int k     = 0;
    int rest  = target.size();
    while (it_t != target.end() && it_s != subtracted.end()) {
      if (*it_t == *it_s) {
        ++it_t;
        ++it_s;
        --rest;
        continue;
      }
      if ((*it_t)->id() < (*it_s)->id()) {
        ++it_t;
        ++k;
        --rest;
      } else {
        ++it_s;
      }
    }
    return k + rest;
  }

  bool QuotientGraph::Node::isIndistinguisheable(const QuotientGraph::NodePtr& other) const {
    auto it1 = _adjacent.begin();
    auto it2 = other->_adjacent.begin();
    while (it1 != _adjacent.end() && it2 != other->_adjacent.end()) {
      if (*it1 == other) {
        ++it1;
        continue;
      }
      if (*it2 == this) {
        ++it2;
        continue;
      }
      if (*it1 != *it2)
        return false;
      ++it1;
      ++it2;
    }
    if (it1 != _adjacent.end() || it2 != other->_adjacent.end())
      return false;

    it1 = _elements.begin();
    it2 = other->_elements.begin();
    while (it1 != _elements.end() && it2 != other->_elements.end()) {
      if (*it1 != *it2)
        return false;
      ++it1;
      ++it2;
    }
    return (it1 == _elements.end() && it2 == other->_elements.end());
  }

  void QuotientGraph::Node::getNeighbors(NodePtrSet& nbr) {
    nbr = _adjacent;
    for (NodePtr e : _elements)
      nbr.insert(e->_adjacent.begin(), e->_adjacent.end());
    setDifferenceInPlace(nbr, _super_variable);
  }

  void QuotientGraph::Node::eliminate(QuotientGraph& graph) {
    assert(!isElement() && "n is an element");

    // get the neighbors in the adjacent list
    for (NodePtr e : _elements)
      _adjacent.insert(e->_adjacent.begin(), e->_adjacent.end());
    setDifferenceInPlace(_adjacent, _super_variable);

    NodePtrSet super_neighbors;
    for (NodePtr n : _adjacent) {
      if (n->_parent_variable == n)
        super_neighbors.insert(n);
    }

    for (NodePtr n : super_neighbors) {
      // a. adjacent update
      setDifferenceInPlace(n->_adjacent, _adjacent);
      setDifferenceInPlace(n->_adjacent, _super_variable);

      // b. element absorbing
      setDifferenceInPlace(n->_elements, _elements);
      n->_elements.insert(this);
    }
    switch (graph._policy) {
      case Exact:
        exactDegreeUpdate(graph, super_neighbors);
        break;
      case External:
        externalDegreeUpdate(graph, super_neighbors);
        break;
      case Approximate:
        approxDegreeUpdate(graph, super_neighbors, graph._nodes.size() - graph._order_idx);
        break;
    }

    if (graph._enable_supervariables)
      superVariableEliminate(graph);

    // convert node to element
    // assert(_super_variable.size()==1 && "super var");
    for (NodePtr n : _super_variable) {
      n->_elimination_order = graph._order_idx++;
      n->_is_element        = true;
      n->_elements.clear();
      n->_degree = -1;
      n->nodeDegreeUpdate(graph);
    }
  }

  void QuotientGraph::Node::approxDegreeUpdate(QuotientGraph& graph,
                                               NodePtrSet& super_neighbors,
                                               int active_matrix_size) {
    // clean previous shit
    for (NodePtr n : super_neighbors) {
      n->_w = -1;
      for (NodePtr e : n->_elements)
        e->_w = -1;
    }

    // do alg 2
    for (NodePtr n : super_neighbors) {
      for (NodePtr e : n->_elements) {
        if (e->_w < 0) {
          e->_w = e->_adjacent.size();
        }
        e->_w -= n->_super_variable.size();
      }
    }
    // compute degrees eq 4
    for (NodePtr n : super_neighbors) {
      int num_adjacent = setDifferenceSum(n->_adjacent, n->_super_variable);
      int num_clique   = setDifferenceSum(_adjacent, n->_super_variable);
      int d            = num_adjacent + num_clique;
      for (NodePtr e : n->_elements) {
        if (e == this)
          continue;
        if (e->_w < 0)
          d += e->_adjacent.size();
        else
          d += e->_w;
      }

      n->_degree = std::min(d, active_matrix_size);
      n->_degree = std::min(n->_degree, n->_old_degree + num_clique);
      n->nodeDegreeUpdate(graph);
    }
  }

  void QuotientGraph::Node::externalDegreeUpdate(QuotientGraph& graph,
                                                 NodePtrSet& super_neighbors) {
    for (NodePtr n : super_neighbors) {
      // c. degree computation
      n->_degree = n->computeExternalDegree();
      // n->_degree=n->computeDegree();
      n->nodeDegreeUpdate(graph);
    }
  }

  void QuotientGraph::Node::exactDegreeUpdate(QuotientGraph& graph, NodePtrSet& super_neighbors) {
    for (NodePtr n : super_neighbors) {
      // c. degree computation
      n->_degree = n->computeExactDegree();
      // n->_degree=n->computeDegree();
      n->nodeDegreeUpdate(graph);
    }
  }

  void QuotientGraph::Node::superVariableEliminate(QuotientGraph& graph) {
    // super variable detection end elimination
    std::vector<std::list<NodePtr>>& hash_buckets = graph._hash_buckets;
    IntSet& hash_ids                              = graph._hash_ids;

    for (NodePtr n : _adjacent) {
      int h = n->computeHash() % graph._nodes.size();
      hash_buckets[h].push_back(n);
      hash_ids.insert(h);
    }

    for (int h : hash_ids) {
      std::list<NodePtr>& bucket = hash_buckets[h];
      for (auto it = bucket.begin(); it != bucket.end(); ++it) {
        NodePtr node_i = *it;
        auto iit       = it;

        ++iit;
        while (iit != bucket.end()) {
          NodePtr node_j = *iit;
          if (node_i->isIndistinguisheable(node_j)) {
            node_i->_super_variable.insert(node_j->_super_variable.begin(),
                                           node_j->_super_variable.end());
            node_i->_degree -= node_j->_super_variable.size();

            node_j->_parent_variable = node_i->_parent_variable;
            node_j->_elements.clear();
            node_j->_super_variable.clear();
            node_j->_adjacent.clear();

            node_j->_degree = -1;
            node_j->nodeDegreeUpdate(graph);
            iit = bucket.erase(iit);
          } else
            ++iit;
        }
        node_i->nodeDegreeUpdate(graph);
      }
      bucket.clear();
    }
    hash_ids.clear();
  }

  void QuotientGraph::Node::print(std::ostream& os) const {
    os << "id: " << id() << std::endl;
    os << "adjacent: ";
    for (NodePtr n : _adjacent) {
      os << n->id() << " ";
    }
    os << std::endl;

    os << "element: ";
    for (NodePtr n : _elements) {
      os << n->id() << " ";
    }
    os << std::endl;
  }

  int QuotientGraph::Node::computeHash() {
    int hash = 0;
    for (NodePtr a : _adjacent) {
      hash += a->_parent_variable->id();
    }
    for (NodePtr e : _elements) {
      hash += e->id();
    }
    return hash;
  }

  void QuotientGraph::Node::addNeighbor(Node* n) {
    assert(!isElement() && "this is an element");
    assert(!n->isElement() && "n is an element!");
    assert(n != this && "n is cannot be his own neighbor");
    _adjacent.insert(n);
    n->_adjacent.insert(this);
  }

  int QuotientGraph::Node::computeExactDegree() {
    NodePtrSet neighbors = _adjacent;
    for (NodePtr e : _elements) {
      assert(e->_is_element);
      neighbors.insert(e->_adjacent.begin(), e->_adjacent.end());
    }
    neighbors.erase(this);
    return neighbors.size();
  }

  int QuotientGraph::Node::computeExternalDegree() {
    int external_degree = 0;
    external_degree     = setDifferenceSum(_adjacent, _super_variable);

    NodePtrSet neighbors;
    for (NodePtr e : _elements) {
      neighbors.insert(e->_adjacent.begin(), e->_adjacent.end());
    }

    external_degree += setDifferenceSum(neighbors, _super_variable);
    return external_degree;
  }

  void QuotientGraph::Node::nodeDegreeUpdate(QuotientGraph& graph) {
    if (_old_degree >= 0) {
      graph._nodes_with_degree[_old_degree].erase(this);
      if (graph._nodes_with_degree[_old_degree].empty())
        graph._degrees.erase(_old_degree);
    }
    _old_degree = _degree;
    if (_degree >= 0) {
      graph._nodes_with_degree[_degree].insert(this);
      graph._degrees.insert(_degree);
    }
  }

  QuotientGraph::QuotientGraph(const IntPairVector& block_indices, int num_nodes) {
    _nodes.resize(num_nodes);
    _nodes_with_degree.resize(num_nodes);
    for (int i = 0; i < num_nodes; ++i) {
      _nodes[i].setId(i);
    }
    for (const IntPair& f : block_indices) {
      if (f.first == f.second)
        continue;
      Node* from_node = &_nodes[f.first];
      Node* to_node   = &_nodes[f.second];
      from_node->addNeighbor(to_node);
      to_node->addNeighbor(from_node);
    }
  }

  void QuotientGraph::setPolicy(Policy policy_) {
    _policy = policy_;
    switch (_policy) {
      case External:
      case Approximate:
        _enable_supervariables = true;
        break;
      case Exact:
        _enable_supervariables = false;
    }
  }

  void QuotientGraph::mdo(IntVector& ordering) {
    for (Node& n : _nodes) {
      switch (_policy) {
        case Exact:
          n._degree = n.computeExactDegree();
          break;
        case External:
        case Approximate:
          n._degree = n.computeExternalDegree();
          break;
      }
      n.nodeDegreeUpdate(*this);
    }
    int num_nodes = _nodes.size();
    _hash_ids.clear();
    _hash_buckets.resize(num_nodes);
    ordering.resize(num_nodes);
    while (!_degrees.empty()) {
      int current_degree = *_degrees.begin();
      Node* n            = *_nodes_with_degree[current_degree].begin();
      n->eliminate(*this);
    }
    for (int i = 0; i < num_nodes; ++i) {
      ordering[_nodes[i].ordering()] = i;
    }
  }
} // namespace srrg2_solver
