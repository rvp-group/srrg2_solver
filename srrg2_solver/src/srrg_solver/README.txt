Here I defined a generic interface for the DENSE least squares problem
To use this stuff you need:

1. /*Defining the state*/

   Specialize and extend the
   class Variable_<EstimateType_, int PerturbationDim>
   to represent your state.

   EstimateType_ should have a template parameter for the scalar
   E.G.
   Isometry3_<Scalar> is a good example

   Example
   class VariableSE3: public Variable_<Isometry3_, 6> {
   public:

      // Implement the method below
      // this adds to the internal estimate variable
      // of type EstimateType_<float>, the perturbation.

      // This action is not required if you use implement autodiff
      void applyPerturbation(const Vector6f& v) {
         _estimate = t2v(v)*_estimate;
      }

      // this clears the initial estimate variable
      void setZero() {
         _estimate.setIdentity();
      }
      
   };
   
   If you use autodiff, in addition to the previous step
   (you still need to declare a "plain" variable type)
   you will extend the class by adding the autodiff functionalities
   to an existing variable as follows

   class VariableSE3AD: public ADVariable_< VariableSE3 > {
   public:

      // add the typedefs below!!!!
      typedef VarableSE3_ VariableType;
      typedef typename ADVariableType::ADPerturbationVectorType ADPerturbationVectorType;
      typedef typename ADVariableType::ADEstimateType ADEstimateType;

      // This action is not required if you use implement autodiff
      // this is equivalent to the upper method, only it uses
      // AD variables
      // the member variable ad_Estimate 
      virtual void applyPerturbationAD(const Vector6<DualValue>& ad_v) {
         _ad_estimate = _ad_estimate*t2v(ad_v);
      }
   };

   
 
2. /* Define the measurements */
 
   Define the measurement(s) of your problem, by deriving the class
   Measurement<int ErrorDim_>
   A problem can contain heterogeneous measurements

   ErrorDim_ is the size of the error vector,
   not necessarily the size of the measurement

   example:
   class MyMeasurement: public Measurement_<3> {
   public:
     Vector3f point1;
     Vector3f point2;
   };

   or

   class MyMeasurementInfo: public MeasurementInformation_<3> {
   public:
     Vector3f point1;
     Vector3f point2;
   };

   
   here 3 is the dimension of the error function;
   The first class does not add an information matrix,
   the second attaches to the measurement an information matrix (3x3 in this case)

3. /* Define Error Functions and Factors*/

  Define your error function(s) (be consistent with the way you
   apply the perturbation)

   NO AUTODIFF
      a. extend the

           class ErrorFunction_<VariableType, ErrorDim>
       
         by

         1. setting the right dimension of the error

         2. overriding
            void setState(const VariableType* s),

         3. adding some variables that are used in the computation of error and jacobian
            (point position, parameters etc)
           
         4. define the function
            void compute();

            that should alter the member variables
            - _e: with the error
            - _J: with the the jacobian
            - _is_valid: with true or false depending on the computation

         5. construct a "Factor" that binds the error function to a measurement type
         
        example:

        class PointErrorFunction: public ErrorFunction_<VariableSE3, 3 > {
        public:
            void setState(const VariableSE3* s){
                 my_state=s->estimate();
            }

            void compute(){
                 Vector3f rp1=X.linear()*p1;
                 _e=X*p1-p2;
                 _J.setZero();
                 _J.block<3,3>(0,0).setIdentity(); // this is constant you can move it in ctor;
                 _J.block<3,3>(0,3)=-skew(rp1);
            }

            Isometry3f X;
            Vector3f p1;
            Vextor3f p2;
        }


        Next we are going to construct a factor out
        A factor is the stuff our solver understands, and we need to wrap the
        error function into it.
        Also a factor (as an error function), can process only measurements of its type

        Do this:
        class PointErrorFactor: public LeastSquaresErrorFactor_<PointErrorFunction, MyMeasurement> {
            inline void applyMeasurement(const MyMeasurement* m) {
                   p1=m->p1;
                   p2=m->p2;
            }
        };


        The factor inherits from the error function.
        you need to "override" a method applyMeasurement
        to "configure" the error function to compute the actual value
        
        


    AUTODIFF
        1. create functor that computes the error
        2. create an error function from the functor
        3. creare a factor 

        
        here you need to create a Functor based on the variable that evaluates the error
        basically, you override the () operator ad follows
        NOte you don't need to handle the increments as this is already done by autodiff
        Remember that all variables used in the computation should have a <DualValuef>
        as scalar.
        To this extent, you can add setter methods that copy the values in
        internal autodiff variables from normal float parameters
        (see setP1 and setP2)

    example:
    
    class PointErrorFunctorAD: public ADErrorFunctor_<VariableSE3 , 3> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      // here we compute our error!
      virtual Vector3_<DualValuef> operator()(const Isometry3_<DualValuef>& T)  override {
        return T*_p1-_p2;
        _is_valid=true;
      }
      
      // these convert an arbitrary type to a scalar, for error eval
      void setP1(const Vector3f& p1_)   {convertMatrix(_p1, p1_);}
      void setP2(const Vector3f& p2_) {convertMatrix(_p2, p2_);}
    protected:
      // parameters << all parameters 
      Vector3_<DualValuef> _p1;
      Vector3_<DualValuef> _p2;
    };


    Ok now you have a functor, we need to construct an error function from it.
    Do it with the following

    typedef ADErrorFunction_<PointErrorFunctorAD> PointErrorFunctionAD;


    Finally we need to wrap the error function into a factor as follows:

        class PointErrorFactor: public LeastSquaresErrorFactorAD_<PointErrorFunctionAD, MyMeasurement> {
            inline void applyMeasurement(const MyMeasurement* m) {
                   setP1(m->p1);
                   setP2(m->p2);
            }
        };

4. Create a solver

   Heck, this is almost automatic.
   The only thing you have to do are:
   1. choosing a "container" that will allow your solver
      to select which factor to use with each measurement
      A default container builds an index based on the typeid

      It's slow as hell.
      All in all the container is a template that implements the following methods

      template <typename FactorType>
      class FactorContainer {
      public:
               FactorType* getFactor(const MeasurementBase* measurement);
      }

      if you handle a single measurement types the thing can be trivial.

   another default container looks into he field measurement->factorId()
   to select the right factor.


   Once you selected a container at your choice you do this

   NO AUTODIFF

   class MySolver: public SolverStandard_<VariableSE3, ContainerType> {
         MySolver() {
                    _factors.addFactor(new PointErrorFactor);
                    _factors.addFactor(new ProjErrorFactor);
                    .....
         }
   }

   class MySolverAD: public SolverStandard_<VariableSE3AD, ContainerType> {
         MySolver() {
                    _factors.addFactor(new PointErrorFactorAD);
                    _factors.addFactor(new ProjErrorFactor);
                    .....
         }
   }

   Yes you can mix AD and NON-AD things, as long as the core variable is the same.

   

   you are done.
Cheers,
        G.
        
