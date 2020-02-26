#pragma once
#include <vector>
#include <limits>

namespace srrg2_solver {
  template <typename T>
  class BehaviorAnalyzer_ {
  public:
    using ContainerType=std::vector<T>;

    BehaviorAnalyzer_(int window_size=5){
      reset(window_size);
    }
    
    void reset(int window_size) {
      _window_size=window_size;
      _buffer.resize(window_size);
      _num_samples=0;
    };
    
    inline const T& min() const {return _min;}
    inline const T& max() const {return _max;}
    inline T range() const {return _max - _min;}
    inline int numSamples() const {return _num_samples;}

    void addSample(const T& s) {
      int next_idx=_num_samples%_window_size;
      _buffer[next_idx]=s;
      ++_num_samples;
      _min = std::numeric_limits<T>::max();
      _max = -std::numeric_limits<T>::max();
      for (int i=0; i<std::min(_window_size,_num_samples); ++i) {
        _min=std::min(_buffer[i], _min);
        _max=std::max(_buffer[i], _max);
      }
    }
  protected:
    int _window_size;
    ContainerType _buffer;
    int _num_samples=0;
    T _min =  std::numeric_limits<T>::max();
    T _max = -std::numeric_limits<T>::max();
  };

}
