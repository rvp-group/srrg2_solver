#pragma once
//! includes all the implementation layers
//! put this at the beginning of a cpp file
//! where you instantiate *all* the types you declared

#include "srrg_solver/solver_core/variable_impl.cpp"
#include "srrg_solver/solver_core/factor_impl.cpp"
#include "srrg_solver/solver_core/ad_error_factor_impl.cpp"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
