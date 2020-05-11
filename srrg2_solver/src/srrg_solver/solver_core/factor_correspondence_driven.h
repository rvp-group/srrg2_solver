#pragma once
#include "factor_base.h"
#include <srrg_data_structures/correspondence.h>
namespace srrg2_solver {
  using namespace srrg2_core;
  /*! @brief Implements a general interface for data driven factors. Outside a
    FactorCorrespondenceDriven_ is visible as a standard FactorBase but in fact it acts like a
    "container" of factors of the same type. This class is a perfect fit for alignment problems
    where you a typically have lots of factors of the same type connected to a single variable.
    Indeed this interface allows to scale this concept to factors with an arbitrary number of
    variables.

    To define your FactorCorrespondenceDriven_ you just need to specify the templates value :
    - FactorBaseType_ the factor that you want to make data driven
    - FixedContainerType_ container of the measurements
    - MovingContainerType_ container of the data that are used togheter with the variables to get a
    prediction
    - CorrespondenceContainerType_ container of associations between Fixed and Moving data
   */
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
    using InformationMatrixType = typename FactorBaseType_::InformationMatrixType; /*!< Information
                                                                                     matrix type */
    using InformationMatrixVector =
      std::vector<InformationMatrixType,
                  Eigen::aligned_allocator<InformationMatrixType>>; /*!< Container of
                                                                      information
                                                                      matrices */

    /*! Set the information matrix container, which should have the same dimension as the Fixed
      container, so that we ensure that for each correspondence we have an associated information
      matrix
      @param[in] information_matrix_vector_ container of information matrices
    */
    void setInformationMatrixVector(const InformationMatrixVector& information_matrix_vector_) {
      _information_matrix_vector = &information_matrix_vector_;
    }
    /*! Set fixed container
      @param[in] fixed_ container of data
    */
    void setFixed(const FixedContainerType& fixed_) {
      _fixed = &fixed_;
    }
    /*! Set moving container
      @param[in] moving_ container of data
    */
    void setMoving(const MovingContainerType& moving_) {
      _moving = &moving_;
    }
    /*! Set correspondence container
      @param[in] correspondence_ container of associations between fixed and moving data
    */
    void setCorrespondences(const CorrespondenceContainerType& correspondences_) {
      _correspondences = &correspondences_;
      setBegin();
    }

    /*! Set the internal iterator to the beginning of the correspondences */
    void setBegin() override {
      _current_it = _correspondences->begin();
    }

    /*! @return True if end of the container if reached */
    bool isEnd() override {
      return _current_it == _correspondences->end();
    }

    /*! @return number of elements to iterate */
    size_t size() override {
      return _correspondences->size();
    }

    /*! @return Current factor element */
    FactorBase*& get() override {
      assert(
        (!_information_matrix_vector || _information_matrix_vector->size() == _fixed->size()) &&
        "FactorCorrespondenceDriven| information matrix vector size mismatch with fixed data");

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

    /*! Increase the internal iterator */
    IteratorInterface_<FactorBase*>& next() override {
      ++_current_it;
      return *this;
    }

    /*! Gets the current factor and increments the iterator
      @param[in] datum current factor
     @return false if at the end of the container
    */
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
    const FixedContainerType* _fixed   = nullptr; /*!< Fixed data container pointer */
    const MovingContainerType* _moving = nullptr; /*!< Moving data container pointer */
    const CorrespondenceContainerType* _correspondences = nullptr;       /*!< Correspondences
                                                                           container pointer */
    const InformationMatrixVector* _information_matrix_vector = nullptr; /*!< Information matrix
                                                                           container pointer */
    CorrespondenceIteratorType _current_it; /*!< Iterator to the current assocation/factor */
    FactorBase* _returned = 0;              /*!< Factor to be returned by the interface */
  };

} // namespace srrg2_solver
