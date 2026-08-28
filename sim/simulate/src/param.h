#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace param
{

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;
    // Used when joystick_type == "lcm": the FSM is then driven by joy_t on
    // this channel instead of by a kernel device.
    std::string joystick_lcm_channel = "JOYSTICK";
    std::string joystick_lcm_url;

    int print_scene_information;

    // Run without a viewer. MuJoCo only needs GL for rendering, so this is the
    // mode for CI, for the robot, and for an X display without GLX.
    int headless = 0;
    // MuJoCo's own side panels. Off by default: on a laptop the two take
    // roughly a third of the window from the thing you are watching.
    int show_left_ui = 0;
    int show_right_ui = 0;

    int enable_elastic_band;
    // Headless only: seconds of sim time after which the band lets go. The GUI
    // binds this to a key; a scripted run needs it on a timer, late enough that
    // the FSM has reached a standing state. <= 0 never releases.
    double band_release_s = 0.0;
    int band_attached_link = 0;

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();
            if (cfg["joystick_lcm_channel"]) joystick_lcm_channel = cfg["joystick_lcm_channel"].as<std::string>();
            if (cfg["joystick_lcm_url"]) joystick_lcm_url = cfg["joystick_lcm_url"].as<std::string>();
            if (cfg["headless"]) headless = cfg["headless"].as<int>();
            if (cfg["show_left_ui"]) show_left_ui = cfg["show_left_ui"].as<int>();
            if (cfg["show_right_ui"]) show_right_ui = cfg["show_right_ui"].as<int>();
            if (cfg["band_release_s"]) band_release_s = cfg["band_release_s"].as<double>();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
        ("headless", po::bool_switch()->notifier([](bool v){ if (v) config.headless = 1; }), "Run without a viewer")
        ("joystick,j", po::value<std::string>(&config.joystick_type),
            "Where the gamepad comes from: xbox / switch read /dev/input directly, "
            "lcm takes joy_t off joystick_lcm_channel (scriptable, works over SSH)")
        ("band-release", po::value<double>()->notifier([](double v){ config.band_release_s = v; }),
            "Headless only: sim seconds before the elastic band lets go (overrides config.yaml)")
        ("ui", po::bool_switch()->notifier([](bool v){ if (v) { config.show_left_ui = 1; config.show_right_ui = 1; } }), "Show MuJoCo's side panels (off by default)")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}