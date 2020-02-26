#pragma once
#define BOSS_REGISTER_AND_INSTANTIATE(class_name)            \
  namespace explicit_instances {                             \
    class_name class_name##_instance;                        \
  }                                                          \
  void class_name##_registerClass() {                        \
    BOSS_REGISTER_CLASS(class_name);                         \
  }                                                          \

#define INSTANTIATE(class_name)                              \
  namespace explicit_instances {                             \
    class_name class_name##_instance;                        \
  }                                                          \

