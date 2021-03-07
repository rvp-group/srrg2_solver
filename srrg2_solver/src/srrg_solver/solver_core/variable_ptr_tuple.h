#pragma once
#include "variable.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  /*! @brief Recursive class to access type of variable i in the tuple */
  template <typename VariablePtrTupleType_, int i>
  struct VariableTypeAt_ {
    using VariableType =
      typename VariableTypeAt_<typename VariablePtrTupleType_::RestTupleType, i - 1>::VariableType;
  };
  /*! @brief Base case of the recursion for VariableTypeAt_ */
  template <typename VariablePtrTupleType_>
  struct VariableTypeAt_<VariablePtrTupleType_, 0> {
    using VariableType = typename VariablePtrTupleType_::VariableType_;
  };

  /*! @brief Recursive class to access pointer to variable i in the tuple */
  template <typename VariablePtrTupleType_, int i>
  struct VariableAt_ {
    static inline typename VariableTypeAt_<VariablePtrTupleType_, i>::VariableType*&
    variable(VariablePtrTupleType_& vtuple) {
      return VariableAt_<typename VariablePtrTupleType_::RestTupleType, i - 1>::variable(vtuple);
    }
  };

  /*! @brief Base case of the recursion for VariableAt_ */
  template <typename VariablePtrTupleType_>
  struct VariableAt_<VariablePtrTupleType_, 0> {
    static inline typename VariablePtrTupleType_::VariableType_*&
    variable(VariablePtrTupleType_& vtuple) {
      return vtuple.variable;
    }
  };
  /*!@brief Variadic variable pointer tuple container */
  template <typename V, typename... VRest>
  struct VariablePtrTuple_ : public VariablePtrTuple_<VRest...> {
    using ThisType      = VariablePtrTuple_<V, VRest...>;
    using RestTupleType = VariablePtrTuple_<VRest...>;
    V* variable; /*!< Pointer to the first variable in the tuple */
    /*!< Number of variables in the tuple */
    static constexpr int size() {
      return sizeof...(VRest) + 1;
    }

    using Id            = typename V::Id;
    using VariableType_ = V;
    /*! Get type of variable in position i */
    template <int i>
    using VariableTypeAt = typename VariableTypeAt_<ThisType, i>::VariableType;

    Id _graph_id = -1;
    /*! Get variable pointer (with the proper type) at position i
     @return Variable pointer
     */
    template <int i>
    VariableTypeAt<i>*& at() {
      return VariableAt_<ThisType, i>::variable(*this);
    }
    /*! Set variable pointer at position i
      @param[in] i index in the tuple
      @param[in] v variable pointer
    */
    void setVariableAt(int i, VariableBase* v) {
      if (i == 0) {
        variable = reinterpret_cast<VariableType_*>(v);
        return;
      }
      RestTupleType::setVariableAt(i - 1, v);
    }
    /*! Get variable pointer at position i
      @param[in] i index of the variable in the tuple
      @return Variable pointer
    */
    VariableBase* variableAt(int i) {
      if (i == 0) {
        return variable;
      }
      return RestTupleType::variableAt(i - 1);
    }
    /*! Get variable pointer (read only) at position i
      @param[in] i index of the variable in the tuple
      @return Variable pointer
    */
    const VariableBase* variableAt(int i) const {
      if (i == 0) {
        return variable;
      }
      return RestTupleType::variableAt(i - 1);
    }
    /*! Get graph id of the variable at index pos in the tuple
     @param[in] pos index in the tuple
     @return Graph id
     */
    inline Id graphId(int pos) const {
      if (pos == 0) {
        return _graph_id;
      }
      return RestTupleType::graphId(pos - 1);
    }
    /*! Set the graph id for the variable at index pos in the tuple
     @param[in] pos index of the variable in the tuple
     @param[in] graph_id
     */
    inline void setGraphId(int pos, int graph_id) {
      if (pos == 0) {
        _graph_id = graph_id;
        return;
      }
      RestTupleType::setGraphId(pos - 1, graph_id);
    }
    /*! Fill the tuple with a container
      @param[in] container the container of variable
      @return Number of dangling variables
    */
    inline int bind(IdVariablePtrContainer& container) {
      int dangling = 1;
      if (_graph_id < 0) {
        throw std::runtime_error("invalid variable id in bind");
      }
      IdVariablePtrContainer::iterator it = container.find(_graph_id);
      if (it != container.end()) {
        VariableBase * v = it.value();
        if (v && v != variable) {
          v->_tainted = true;
        }
        variable = dynamic_cast<V*>(v);
        assert(variable && "type mismatch in bind");
        dangling = 0;
      }
      return dangling + RestTupleType::bind(container);
    }
    /*! Get the perturbation dimension of the i-th variable
     @return Perturbation dimension
     */
    template <int i>
    static constexpr int perturbationDim() {
      return (i == 0) ? V::PerturbationDim : RestTupleType::template perturbationDim<i - 1>();
    }
    /*! Get perturbation offset of the i-th variable
      @return Perturbation offset
    */
    template <int i>
    static constexpr int perturbationOffset() {
      return (i == 0) ? 0
                      : V::PerturbationDim + RestTupleType::template perturbationOffset<i - 1>();
    }

    static constexpr int NumVariables         = sizeof...(VRest) + 1;
    static constexpr int TotalPerturbationDim = perturbationOffset<NumVariables>();
    inline bool tainted() const {
      return variable->tainted() || RestTupleType::tainted();
    }
    inline void untaint() {
      variable->_taint = false;
      RestTupleType::untaint();
    }
  };
  /*! @brief Base case for variatic template recursion - See VariablePtrTuple_ */
  template <typename V>
  struct VariablePtrTuple_<V> {
    V* variable = nullptr;

    using ThisType      = VariablePtrTuple_<V>;
    using VariableType_ = V;

    template <int i>
    using VariableTypeAt = typename VariableTypeAt_<ThisType, i>::VariableType;

    using Id = typename V::Id;

    static constexpr int size() {
      return 1;
    }

    Id _graph_id = -1;

    template <int i>
    VariableTypeAt<i>*& at() {
      return VariableAt_<ThisType, i>::variable(*this);
    }

    void setVariableAt(int i, VariableBase* v) {
      if (i == 0) {
        variable = reinterpret_cast<VariableType_*>(v);
        return;
      }
      throw std::runtime_error("cane, variable index of bounds");
    }

    VariableBase* variableAt(int i) {
      if (i == 0) {
        return variable;
      }
      throw std::runtime_error("rutto, variable index of bounds");
    }

    const VariableBase* variableAt(int i) const {
      if (i == 0) {
        return variable;
      }
      throw std::runtime_error("rutto, variable index of bounds");
    }

    inline Id graphId(int pos) const {
      if (pos == 0) {
        return _graph_id;
      }
      throw std::runtime_error("id index of bounds");
    }

    inline void setGraphId(int pos, int graph_id) {
      if (pos == 0) {
        _graph_id = graph_id;
        return;
      }
      throw std::runtime_error("id index of bounds");
    }

    inline int bind(IdVariablePtrContainer& container) {
      if (_graph_id < 0) {
        throw std::runtime_error("invalid variable id in bind");
      }
      IdVariablePtrContainer::iterator it = container.find(_graph_id);
      if (it != container.end()) {
        VariableBase* v = it.value();
        if (v && v != variable) {
          v->_tainted = true;
        }
        variable = dynamic_cast<V*>(v);
        assert(variable && "type mismatch in bind");
        return 0;
      }
      return 1;
    }

    template <int i>
    static constexpr int perturbationDim() {
      return V::PerturbationDim;
    }

    template <int i>
    static constexpr int perturbationOffset() {
      return (i == 0) ? 0 : V::PerturbationDim;
    }

    inline bool tainted() const {
      return variable->tainted();
    }
    inline void untaint() {
      variable->_taint = false;
    }
    static constexpr int NumVariables         = 1;
    static constexpr int TotalPerturbationDim = V::PerturbationDim;
  };

} // namespace srrg2_solver
