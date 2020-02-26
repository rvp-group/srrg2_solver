#pragma once
#include "factor_base.h"
#include <memory>
#include <srrg_data_structures/correspondence.h>
#include <srrg_data_structures/indexed_container_interface.h>
namespace srrg2_solver {
  using namespace srrg2_core;

  template <typename FactorBaseType_, typename CorrespondenceContainerType_ = CorrespondenceVector>

  class FactorCorrespondenceDrivenDynamic_ : public FactorBaseType_ {
  public:
    using CorrespondenceContainerType = CorrespondenceContainerType_;
    using CorrespondenceIteratorType  = typename CorrespondenceContainerType::const_iterator;
    using FixedType                   = typename FactorBaseType_::FixedType;
    using MovingType                  = typename FactorBaseType_::MovingType;
    using BaseFactorType              = FactorBaseType_;

    virtual ~FactorCorrespondenceDrivenDynamic_() {
    }

    template <typename FixedContainerType>
    void setFixed(const FixedContainerType& fixed_) {
      _fixed_container.reset(new IndexedContainerInterface_<FixedContainerType>(fixed_));
    }

    template <typename MovingContainerType>
    void setMoving(const MovingContainerType& moving_) {
      _moving_container.reset(new IndexedContainerInterface_<MovingContainerType>(moving_));
    }

    void setCorrespondences(const CorrespondenceContainerType& correspondences_) {
      _correspondences = &correspondences_;
      setBegin();
    }

    /*Iterator Interface, for data driven factors*/
    // sets the iterator to the beginning
    void setBegin() override {
      _current_it = _correspondences->begin();
    }

    // true if end is reached
    bool isEnd() override {
      return _current_it == _correspondences->end();
    }

    // number of elements to iterate
    size_t size() override {
      return _correspondences->size();
    }

    // gets the current element
    FactorBase*& get() override {
      const int fixed_idx  = _current_it->fixed_idx;
      const int moving_idx = _current_it->moving_idx;

      if (fixed_idx < 0 || moving_idx < 0) {
        _returned = 0;
        return _returned;
      }
      FactorBaseType_::setFixed(_fixed_container->const_at(fixed_idx));
      FactorBaseType_::setMoving(_moving_container->const_at(moving_idx));
      _returned = this;
      return _returned;
    }

    // increments the iterator
    IteratorInterface_<FactorBase*>& next() override {
      ++_current_it;
      return *this;
    }

    // gets the next and increments the pointer
    // returns false if at the end of the container
    virtual bool getNext(FactorBase*& datum) {
      datum = 0;
      if (_current_it == _correspondences->end()) {
        return false;
      }
      const int fixed_idx  = _current_it->fixed_idx;
      const int moving_idx = _current_it->moving_idx;
      if (fixed_idx >= 0 && moving_idx >= 0) {
        FactorBaseType_::setFixed(_fixed_container->const_at(fixed_idx));
        FactorBaseType_::setMoving(_moving_container->const_at(moving_idx));
        datum = this;
      }
      ++_current_it;
      return true;
    }

  protected:
    std::unique_ptr<IndexedContainerInterfaceBase_<FixedType>> _fixed_container;
    std::unique_ptr<IndexedContainerInterfaceBase_<MovingType>> _moving_container;
    const CorrespondenceContainerType* _correspondences = 0;
    CorrespondenceIteratorType _current_it;
    FactorBase* _returned = 0;
  };

} // namespace srrg2_solver
