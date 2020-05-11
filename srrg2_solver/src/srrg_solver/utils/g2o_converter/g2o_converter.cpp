#include <srrg_system_utils/shell_colors.h>

#include "g2o_converter.h"

namespace srrg2_solver {

  std::string G2OConverter::CLASS_NAME("G2OConverter::");
  size_t G2OConverter::_g2o_parameters_graph_id_offset = 1e7;

//! @brief macro to add a conversion action in the system
//!        specify only the action type and bad things happen in the proper way
#define ADD_ACTION(ACTION_TYPE)                                                             \
  ACTION_TYPE* action_##ACTION_TYPE = new ACTION_TYPE(_boss_graph, this);                   \
  if (_action_pull.count(action_##ACTION_TYPE)) {                                           \
    throw std::runtime_error(CLASS_NAME + "ADD_ACTION|adding twice the same action, exit"); \
  }                                                                                         \
  _action_pull.insert(action_##ACTION_TYPE);

//! @brief macro that registers all the added actions in the conversion system.
//!        THIS SHOULD BE CALLED ONLY ONCE AFTER ALL THE ADD_ACTIONS
#define REGISTER_ALL_ACTIONS                                                             \
  for (ConverterActionBase * a : _action_pull) {                                         \
    if (_tag_action_map.count(a->tag()) || _hash_action_map.count(a->hash())) {          \
      throw std::runtime_error(CLASS_NAME + "REGISTER_ALL_ACTIONS|action [" + a->tag() + \
                               "] already registered");                                  \
    }                                                                                    \
    _tag_action_map.insert(std::make_pair(a->tag(), a));                                 \
    _hash_action_map.insert(std::make_pair(a->hash(), a));                               \
  }

//! @brief this macro adds a param action. it add the conversion action in the system
//!        and specifies also the type of factor that will use this parameter
#define ADD_PARAM_ACTION(ACTION_TYPE, FACTOR_TYPE)                                          \
  ACTION_TYPE* action_##ACTION_TYPE = new ACTION_TYPE(_boss_graph, this);                   \
  if (_action_pull.count(action_##ACTION_TYPE)) {                                           \
    throw std::runtime_error(CLASS_NAME + "ADD_ACTION|adding twice the same action, exit"); \
  }                                                                                         \
  _action_pull.insert(action_##ACTION_TYPE);                                                \
  const uint64_t factor_typeid = typeid(FACTOR_TYPE).hash_code();                           \
  const std::string& param_tag = action_##ACTION_TYPE->tag();                               \
  if (_factor_hash_param_tag_map.count(factor_typeid)) {                                    \
    throw std::runtime_error(CLASS_NAME + "ADD_PARAM_ACTION|factor [" +                     \
                             std::to_string(factor_typeid) + "] already registered");       \
  }                                                                                         \
  _factor_hash_param_tag_map.insert(std::make_pair(factor_typeid, param_tag));

  void G2OConverter::_registerConverterActions() {
    // ia you cannot register both AD and non-AD actions.
    // ia therefore you have to choose, I would prefer non AD things if present
    ADD_ACTION(ConverterActionTagFixed);

    // ia 2d stuff, here you can choose between ad and non ad stuff
    ADD_ACTION(ConverterActionVariableSE2Right);
    ADD_ACTION(ConverterActionVariablePoint2);
    ADD_ACTION(ConverterActionSE2PosePoseGeodesicErrorFactor);
    ADD_ACTION(ConverterActionSE2PosePointErrorFactor);
    ADD_ACTION(ConverterActionSE2PosePointBearingErrorFactor);

    // ia chordal stuff
    ADD_ACTION(ConverterActionVariableSE3EulerLeft);
    ADD_ACTION(ConverterActionSE3PosePoseChordalEulerLeftErrorFactor);
    // ia standard pgo
    ADD_ACTION(ConverterActionVariableSE3QuaternionRight);
    ADD_ACTION(ConverterActionSE3PosePoseGeodesicErrorFactor)
    // ia standard pose point
    ADD_ACTION(ConverterActionVariablePoint3);
    ADD_ACTION(ConverterActionSE3PosePointOffsetErrorFactor);
    // ia matchables
    ADD_ACTION(ConverterActionVariableMatchable)
    ADD_ACTION(ConverterActionSE3PoseMatchableErrorFactor);
    // ia param for pose point
    ADD_PARAM_ACTION(ConverterActionSE3PoseOffset, SE3PosePointOffsetErrorFactor);

    REGISTER_ALL_ACTIONS;
  }

  G2OConverter::G2OConverter() {
    _boss_graph = FactorGraphPtr(new FactorGraph());

    solver_registerTypes();
    variables_and_factors_2d_registerTypes();
    variables_and_factors_3d_registerTypes();
    variables_and_factors_calib_registerTypes();

    _registerConverterActions();
  }

  G2OConverter::~G2OConverter() {
    std::cerr << CLASS_NAME + "~G2OConverter|deleting all registered actions" << std::endl;
    for (auto it = _action_pull.begin(); it != _action_pull.end(); ++it) {
      delete *it;
    }
  }

  const size_t G2OConverter::registerG2OParameterId(const size_t& g2o_parameter_graph_id_) {
    if (_g2o_parameters_graph_id_map.count(g2o_parameter_graph_id_)) {
      throw std::runtime_error(CLASS_NAME + "registerG2OParameterId|parameter with id [" +
                               std::to_string(g2o_parameter_graph_id_) +
                               "] already registered, exit");
    }

    const size_t boss_parameter_graph_id = _g2o_parameters_graph_id_offset++;
    _g2o_parameters_graph_id_map.insert(
      std::make_pair(g2o_parameter_graph_id_, boss_parameter_graph_id));
    return boss_parameter_graph_id;
  }

  void G2OConverter::loadGraph(const std::string& input_graph_filename_) {
    if (input_graph_filename_.empty()) {
      throw std::runtime_error(CLASS_NAME + "loadGraph|invalid filename, exit");
    }

    // ia check if we are reading from boss or from g2o
    if (input_graph_filename_.substr(input_graph_filename_.find_last_of(".")) == ".g2o") {
      // ia set the right conversion modality
      _mode = ConversionDirection::G2O2Boss;

      // ia open the stream and read from g2o file, populating the graph.
      std::ifstream graph_stream(input_graph_filename_);
      _loadG2OGraph(graph_stream);

      // ia close input stream
      graph_stream.close();
    } else if (input_graph_filename_.substr(input_graph_filename_.find_last_of(".")) == ".boss") {
      // ia set the right conversion modality
      _mode = ConversionDirection::Boss2G2O;

      // ia here reading is simple, just read the graph
      _boss_graph.reset();
      _boss_graph = FactorGraph::read(input_graph_filename_);

      // ia now analyze the graph and populate strange conversion tables
      const auto& factors = _boss_graph->factors();
      for (auto e_it = factors.begin(); e_it != factors.end(); ++e_it) {
        const uint64_t e_hash  = typeid(*(e_it.value())).hash_code();
        const auto param_tuple = _factor_hash_param_tag_map.find(e_hash);
        if (param_tuple != _factor_hash_param_tag_map.end()) {
          const int64_t& offset_variable_id = e_it.value()->variable(2)->graphId();
          _param_id_param_tag_map.insert(std::make_pair(offset_variable_id, param_tuple->second));
        }
      }
    }

    // ia log something
    std::cerr << CLASS_NAME + "loadGraph|"
              << "graph has [" << FG_YELLOW(_boss_graph->variables().size()) << "] variables and "
              << "[" << FG_YELLOW(_boss_graph->factors().size()) << "] edges\n";
  }

  void G2OConverter::writeGraph(const std::string& output_graph_filename_) {
    if (output_graph_filename_.empty()) {
      throw std::runtime_error(CLASS_NAME + "writeGraph|invalid filename, exit");
    }

    if (_boss_graph == nullptr) {
      throw std::runtime_error(CLASS_NAME + "writeGraph|invalid graph, exit");
    }

    switch (_mode) {
      // ia simple case: writing in boss is done automatically. graph should have been populated
      // ia while reading so it's already there
      case G2OConverter::ConversionDirection::G2O2Boss: {
        if (output_graph_filename_.substr(output_graph_filename_.find_last_of(".")) != ".boss") {
          std::cerr << CLASS_NAME + "writeGraph|WARINING extension does not match conversion type"
                    << std::endl;
        }
        //        _boss_graph->bindFactors();
        _boss_graph->write(output_graph_filename_);
        break;
      }

      // ia other case, we have to explicitely write the thing
      case G2OConverter::ConversionDirection::Boss2G2O: {
        if (output_graph_filename_.substr(output_graph_filename_.find_last_of(".")) != ".g2o") {
          std::cerr << CLASS_NAME + "writeGraph|WARINING extension does not match conversion type"
                    << std::endl;
        }

        std::ofstream stream(output_graph_filename_);
        _writeG2OGraph(stream);
        stream.close();
        break;
      }

      // ia invalid and default, crashes
      case G2OConverter::ConversionDirection::Invalid:
      default:
        throw std::runtime_error(CLASS_NAME + "writeGraph|invalid conversion modality");
        break;
    }
  }

  void G2OConverter::_loadG2OGraph(std::ifstream& f_stream_) {
    // ia if modality is not good exit
    if (_mode != G2OConverter::ConversionDirection::G2O2Boss) {
      throw std::runtime_error(CLASS_NAME + "_loadG2OGraph|invalid modality");
    }

    // ia read the stream line by line
    size_t line_counter = 0;
    std::string line;
    std::string element_tag;
    while (getline(f_stream_, line)) {
      std::stringstream ss(line);

      // ia read the tag and check that has been registered
      ss >> element_tag;
      if (_tag_action_map.count(element_tag) == 0) {
        throw std::runtime_error(CLASS_NAME + "_loadG2OGraph|unregistered tag [" + element_tag +
                                 "]");
      }
      // ia find the proper action delegate to read this tag
      ConverterActionBase* action = _tag_action_map.at(element_tag);
      assert(action && "loadG2OGraph|invalid action");
      action->readFromG2O(ss);

      ++line_counter;
    }
  }

  //! @brief helper for writing a g2o file
  void G2OConverter::_writeG2OGraph(std::ofstream& f_stream_) {
    // ia if not in the proper modality exit
    if (_mode != G2OConverter::ConversionDirection::Boss2G2O) {
      throw std::runtime_error(CLASS_NAME + "_writeG2OGraph|invalid modality");
    }

    if (_boss_graph == nullptr) {
      throw std::runtime_error(CLASS_NAME + "_writeG2OGraph|invalid graph pointer");
    }

    // ia take graph entries
    const auto& variables = _boss_graph->variables();
    const auto& factors   = _boss_graph->factors();

    // ia first we write variables
    for (auto v_it = variables.begin(); v_it != variables.end(); ++v_it) {
      // ia take vertex hash and check that is registered in the converter
      const uint64_t v_hash = typeid(*(v_it.value())).hash_code();
      if (_hash_action_map.count(v_hash) == 0) {
        throw std::runtime_error(CLASS_NAME + "_writeG2OGraph|unregistered hash [" +
                                 std::to_string(v_hash) + "]");
      }

      // ia find the proper action delegate to write this hash
      ConverterActionBase* action = _hash_action_map.at(v_hash);
      assert(action && "writeG2OGraph|invalid action");

      // ia write the tag
      std::string tag = "";
      if (_param_id_param_tag_map.count(v_it.value()->graphId()) != 0) {
        // ia call the param action related to this vertex
        tag = _param_id_param_tag_map.at(v_it.value()->graphId());
      } else {
        tag = action->tag();
      }

      f_stream_ << tag << " ";
      action->writeFromBoss(f_stream_, (void*) v_it.value());
      f_stream_ << "\n";
    }

    // ia then we write the factors
    for (auto e_it = factors.begin(); e_it != factors.end(); ++e_it) {
      // ia take factor hash and check that is registered in the converter
      const uint64_t e_hash = typeid(*(e_it.value())).hash_code();
      if (_hash_action_map.count(e_hash) == 0) {
        throw std::runtime_error(CLASS_NAME + "_writeG2OGraph|unregistered hash [" +
                                 std::to_string(e_hash) + "]");
      }

      // ia find the proper action delegate to write this hash
      ConverterActionBase* action = _hash_action_map.at(e_hash);
      assert(action && "writeG2OGraph|invalid action");

      // ia write the tag
      f_stream_ << action->tag() << " ";
      action->writeFromBoss(f_stream_, (void*) e_it.value());
      f_stream_ << "\n";
    }
  }

} /* namespace srrg2_solver */
