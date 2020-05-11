#pragma once
#include <iostream>
#include <list>
#include <set>
#include <vector>

namespace srrg2_solver {
  /*! @brief Quotient graph, which is used to determine the block ordering in a sparse matrix
    that minimize the fill-in of its factorization */
  class QuotientGraph {
  public:
    enum Policy { Approximate, External, Exact };

    using IntPair       = std::pair<int, int>;
    using IntPairVector = std::vector<IntPair>;
    using IntVector     = std::vector<int>;

    struct Node;
    using NodeVector = std::vector<Node>;
    struct NodePtrComparator {
      inline bool operator()(const Node* a, const Node* b) const {
        return a->id() < b->id();
      }
    };
    using NodePtrSet       = std::set<Node*, NodePtrComparator>;
    using NodePtrSetVector = std::vector<NodePtrSet>;
    using IntSet           = std::set<int>;
    using NodePtr          = Node*;

    struct Node {
      friend class QuotientGraph;
      Node(int id_ = -1) : _id(id_), _parent_variable(this) {
        _super_variable.insert(this);
      }

      inline int id() const {
        return _id;
      }
      inline void setId(int id_) {
        _id = id_;
        _super_variable.insert(this);
      }
      inline int degree() const {
        return _degree;
      }
      inline int ordering() const {
        return _elimination_order;
      }
      void addNeighbor(Node* n);

    protected:
      inline bool isElement() const {
        return _is_element;
      }
      void getNeighbors(NodePtrSet& neighbors_);
      void superVariableEliminate(QuotientGraph& graph);
      void eliminate(QuotientGraph& graph);
      void print(std::ostream& os) const;
      int computeExactDegree();
      int computeExternalDegree();
      int computeHash();
      void nodeDegreeUpdate(QuotientGraph& graph);
      void
      approxDegreeUpdate(QuotientGraph& graph, NodePtrSet& super_neighbors, int active_matrix_size);
      void exactDegreeUpdate(QuotientGraph& graph, NodePtrSet& super_neighbors);
      void externalDegreeUpdate(QuotientGraph& graph, NodePtrSet& super_neighbors);
      bool isIndistinguisheable(const NodePtr& other) const;

      int _id                = -1;
      int _degree            = -1;
      int _old_degree        = -1;
      int _elimination_order = -1;
      NodePtrSet _adjacent;
      NodePtrSet _elements;
      NodePtrSet _super_variable;
      bool _is_element = false;
      NodePtr _parent_variable;
      int _w = -1; // le/lp according to paper of tim davis
    };

    inline NodeVector& nodes() {
      return _nodes;
    }

    inline Policy policy() const {
      return _policy;
    }

    void setPolicy(Policy policy_);

    inline bool superVaribleEnabled() {
      return _enable_supervariables;
    }

  public:
    QuotientGraph(const IntPairVector& block_indices, int num_nodes);

    void mdo(IntVector& ordering);

  protected:
    NodePtrSetVector _nodes_with_degree;
    IntSet _degrees;
    NodeVector _nodes;
    int _order_idx = 0;
    IntSet _hash_ids;
    std::vector<std::list<NodePtr>> _hash_buckets;

    Policy _policy              = Approximate;
    bool _enable_supervariables = true;
  };
} // namespace srrg2_solver
