#pragma once
#include "g2o_converter_actions.h"
#include "srrg_solver/variables_and_factors/types_2d/instances.h"
#include "srrg_solver/variables_and_factors/types_calib/instances.h"
#include "srrg_solver/solver_core/instances.h"

namespace srrg2_solver {

  //! @brief simple converter from/to g2o data.
  class G2OConverter {
  public:
    enum class ConversionDirection { Invalid, Boss2G2O, G2O2Boss };

    G2OConverter();
    virtual ~G2OConverter();

    //! @brief loads a graph - either g2o or boss, check is done inside
    //!        IMPORTANT: filename must contain extension
    void loadGraph(const std::string& input_graph_filename_);

    //! @brief writes a graph - either g2o or boss, check is done inside
    //!        IMPORTANT: filename must contain extension
    void writeGraph(const std::string& output_graph_filename_);

    //! @brief inline function that returns the loaded graph
    const FactorGraphPtr& factorGraph() const {
      return _boss_graph;
    }

    //! @brief this registers the original g2o parameter graph id in the system and returns the
    //! assigned graph id for such parameter
    const size_t registerG2OParameterId(const size_t& g2o_parameter_graph_id_);

    //! @brief get the boss parameter graph id from the g2o one
    inline const size_t& getBossParameterId(const size_t& g2o_parameter_graph_id_) {
      if (!_g2o_parameters_graph_id_map.count(g2o_parameter_graph_id_)) {
        throw std::runtime_error(CLASS_NAME + "getBossParameterId|g2o parameter with graph id [" +
                                 std::to_string(g2o_parameter_graph_id_) +
                                 "] is not registered in the system, exit");
      }

      return _g2o_parameters_graph_id_map[g2o_parameter_graph_id_];
    }

  protected:
    //! @brief initilizes the converter, registering all the known actions
    void _registerConverterActions();

    //! @brief helper for reading a g2o file
    void _loadG2OGraph(std::ifstream& f_stream_);

    //! @brief helper for writing a g2o file
    void _writeG2OGraph(std::ofstream& f_stream_);

  protected:
    //! @brief conversion modality
    ConversionDirection _mode = ConversionDirection::Invalid;
    //! @brief original factor graph
    FactorGraphPtr _boss_graph = nullptr;
    //! @brief map to convert g2o->boss
    std::unordered_map<std::string, ConverterActionBase*> _tag_action_map;
    //! @brief map to convert boss->g2o
    std::unordered_map<uint64_t, ConverterActionBase*> _hash_action_map;
    //! @brief pull of all recognized actions
    std::set<ConverterActionBase*> _action_pull;

    std::map<int64_t, std::string> _param_id_param_tag_map;
    //    std::map<uint64_t, int64_t> factor_hash_param_id_map;
    std::map<uint64_t, std::string> _factor_hash_param_tag_map;

    //! @brief graph id offset when converting parameters
    static size_t _g2o_parameters_graph_id_offset;
    //! @brief map that contains <g2o param graph id, boss param graph id>
    std::map<size_t, size_t> _g2o_parameters_graph_id_map;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    static std::string CLASS_NAME;
  };

} /* namespace srrg2_solver */
