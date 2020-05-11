#pragma once
#include "factor_base.h"
#include <memory>
#include <srrg_data_structures/correspondence.h>
#include <srrg_data_structures/indexed_container_interface.h>
namespace srrg2_solver {
  using namespace srrg2_core;
  /*! @brief Implements a general interface for data driven factors. As the
    FactorCorrespondenceDriven_ this factor is visible as a standard FactorBase but in fact it acts
    like a "container" of factors of the same type. The main difference in this case is that you
    don't have to specificy the containers for the fixed/moving data types. Those containers can be
    of arbitrary type that might change during execution. This feature is managed auto-magically by
    the IndexContainerInterface_ (available in srrg2_core) To define your
    FactorCorrespondenceDrivenDynamic_ you just need to specify the templates value :
    - FactorBaseType_ the factor that you want to make data driven
    - CorrespondenceContainerType_ container of associations between Fixed and Moving data

    In this case the FactorBaseType_ must contain :
    - using FixedType=SomethingFixed (the type of the measurements)
    - using MovingType=SomethingMoving (the type of the data used togheter with the variables to
    determine the prediction)

    Which are used by this interface to determine the types of the data.
  */
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
    /*! Set a fixed container of arbitrary type
     @param[in] fixed_ container which must compatible with IndexedContainerInterface_
     */
    template <typename FixedContainerType>
    void setFixed(const FixedContainerType& fixed_) {
      _fixed_container.reset(new IndexedContainerInterface_<FixedContainerType>(fixed_));
    }

    /*! Set a moving container of arbitrary type
     @param[in] moving_ container which must compatible with IndexedContainerInterface_
     */
    template <typename MovingContainerType>
    void setMoving(const MovingContainerType& moving_) {
      _moving_container.reset(new IndexedContainerInterface_<MovingContainerType>(moving_));
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
    std::unique_ptr<IndexedContainerInterfaceBase_<FixedType>> _fixed_container;   /*!< Unique
                                                                                     pointer
                                                                                     to the
                                                                                     indexed
                                                                                     fixed
                                                                                     container
                                                                                     type */
    std::unique_ptr<IndexedContainerInterfaceBase_<MovingType>> _moving_container; /*!< Unique
                                                                                    pointer
                                                                                    to the
                                                                                    indexed
                                                                                    moving
                                                                                    container
                                                                                    type
                                                                                    */
    const CorrespondenceContainerType* _correspondences = 0; /*!< Pointer to the correspondence
                                                               container */

    CorrespondenceIteratorType _current_it; /*!< Iterator to the current assocation/factor */
    FactorBase* _returned = 0;              /*!< Factor to be returned by the interface */
  };
} // namespace srrg2_solver
