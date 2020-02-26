#pragma once
#include "factor_base.h"
#include <srrg_data_structures/correspondence.h>
namespace srrg2_solver {
  using namespace srrg2_core;

  template <typename FactorBaseType_,
            typename FixedContainerType_,
            typename MovingContainerType_,
            typename CorrespondenceContainerType_ = CorrespondenceVector>

  class FactorCorrespondenceDriven_ : public FactorBaseType_ {
  public:
    using CorrespondenceContainerType = CorrespondenceContainerType_;
    using CorrespondenceIteratorType  = typename CorrespondenceContainerType::const_iterator;
    using FixedContainerType          = FixedContainerType_;
    using MovingContainerType         = MovingContainerType_;
    using FixedItemType               = typename FixedContainerType::value_type;
    using MovingItemType              = typename MovingContainerType::value_type;
    using BaseFactorType              = FactorBaseType_;
    using InformationMatrixType       = typename FactorBaseType_::InformationMatrixType;
    using InformationMatrixVector =
      std::vector<InformationMatrixType, Eigen::aligned_allocator<InformationMatrixType>>;

    void setInformationMatrixVector(const InformationMatrixVector& information_matrix_vector_) {
      _information_matrix_vector = &information_matrix_vector_;
    }

    void setFixed(const FixedContainerType& fixed_) {
      _fixed = &fixed_;
    }

    void setMoving(const MovingContainerType& moving_) {
      _moving = &moving_;
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
      assert(
        (!_information_matrix_vector || _information_matrix_vector->size() == _fixed->size()) &&
        "FactorCorrespondenceDriven| information matrix vector size mismatch with fixed scene");

      const int fixed_idx  = _current_it->fixed_idx;
      const int moving_idx = _current_it->moving_idx;

      if (fixed_idx < 0 || moving_idx < 0) {
        _returned = 0;
        return _returned;
      }
      FactorBaseType_::setFixed(_fixed->at(fixed_idx));
      FactorBaseType_::setMoving(_moving->at(moving_idx));
      if (_information_matrix_vector) {
        FactorBaseType_::setInformationMatrix(_information_matrix_vector->at(fixed_idx));
      }
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
      assert(
        (!_information_matrix_vector || _information_matrix_vector->size() == _fixed->size()) &&
        "FactorCorrespondenceDriven| information matrix vector size mismatch with fixed scene");
      datum = 0;
      if (_current_it == _correspondences->end()) {
        return false;
      }
      const int fixed_idx  = _current_it->fixed_idx;
      const int moving_idx = _current_it->moving_idx;

      if (fixed_idx >= 0 && moving_idx >= 0) {
        FactorBaseType_::setFixed(_fixed->at(fixed_idx));
        FactorBaseType_::setMoving(_moving->at(moving_idx));
        if (_information_matrix_vector) {
          FactorBaseType_::setInformationMatrix(_information_matrix_vector->at(fixed_idx));
        }
        datum = this;
      }
      ++_current_it;
      return true;
    }

  protected:
    const FixedContainerType* _fixed                          = 0;
    const MovingContainerType* _moving                        = 0;
    const CorrespondenceContainerType* _correspondences       = 0;
    const InformationMatrixVector* _information_matrix_vector = 0;
    CorrespondenceIteratorType _current_it;
    FactorBase* _returned = 0;
  };

} // namespace srrg2_solver
