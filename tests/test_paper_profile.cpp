#include "tests/test_support.h"
#include <filesystem>
#include <map>
#include <yaml-cpp/yaml.h>
#include "systems/robot_factory.h"
using namespace rrtpff;
int main(int argc, char **argv) {
    return runTest([&] {
        require(argc == 2, "source directory required");
        const std::filesystem::path root = argv[1];
        std::map<std::string,std::string> planner_by_system;
        for (const char *name : {"uni_bugtrap", "uni_wall", "quad2d_bugtrap", "quad2d_hole",
                                 "quad3d_block", "quad3d_window"}) {
            const auto file = root / "benchmark/config" / (std::string(name) + ".yaml");
            const auto c = YAML::LoadFile(file.string());
            require(c["profile"].as<std::string>() == "flask_strict", "explicit strict profile");
            require(c["numeric"]["profile"].as<std::string>() == "float32_production", "production precision");
            require(c["planner"]["max_nodes"].as<int>()==0 && c["planner"]["max_iterations"].as<int>()==0,
                    "normal limits unlimited");
            require(dfMode(c) == "paper_r2", "r2 default");
            require(c["sampling"]["guidance_probability"].as<double>() == 0, "no map guidance");
            validatePaperFairConfig(c);
            const auto full = YAML::LoadFile((root / "benchmark/config/full_df" / file.filename()).string());
            require(dfMode(full) == "full_df", "retained full mode");
            validatePaperFairConfig(full);
            require(!c["simd"]["width"] && !c["simd"]["enabled"],
                    "fixed AVX2 float8 backend is not a per-case option");
            require(c["pff"]["defer_trajectories"].as<bool>(), "all six deferred curves");
            require(c["pff"]["additional_time_samples"].as<int>() == 0, "no uniform time samples");
            require(c["pff"]["time_policy"].as<std::string>() == "optimal_only", "strict T*-only default");
            require(c["planner"]["stop_at_first_solution"].as<bool>() &&
                    !c["planner"]["rewire_before_first_solution"].as<bool>(), "latency flags");
            require(!c["postprocess"]["shortcut"].as<bool>(), "search-only default");
            const auto anytime_file=root/"benchmark/config/anytime"/file.filename();
            const auto anytime=YAML::LoadFile(anytime_file.string());
            require(!anytime["planner"]["stop_at_first_solution"].as<bool>() &&
                    anytime["planner"]["rewire_before_first_solution"].as<bool>(), "anytime flags");
            auto normalized=YAML::Clone(anytime["planner"]);
            normalized["stop_at_first_solution"]=true;normalized["rewire_before_first_solution"]=false;
            require(YAML::Dump(normalized)==YAML::Dump(c["planner"]), "only two planner flags differ");
            for(const char* section:{"pff","sampling","planar_search","simd","geometry","validation",
                                     "postprocess","lifting","df_limits","numeric","df"}) {
                if(c[section]) require(YAML::Dump(c[section])==YAML::Dump(anytime[section]),"protocol parameters identical");
                else require(!anytime[section],"no extra protocol section");
            }
            require(std::filesystem::weakly_canonical(file.parent_path()/c["dynobench"]["source"].as<std::string>())==
                    std::filesystem::weakly_canonical(anytime_file.parent_path()/anytime["dynobench"]["source"].as<std::string>()),
                    "anytime relative source path targets same data");
            require(c["validation"]["max_step"].as<double>() == .005,
                    "specified discrete resolution preserved");
            require(!c["planner"]["top_k"], "Top-K removed");
            const std::string system = c["system"].as<std::string>();
            const auto planner_dump=YAML::Dump(c["planner"]);
            if(planner_by_system.count(system)) require(planner_by_system.at(system)==planner_dump,"same-system maps share planner parameters");
            else planner_by_system[system]=planner_dump;
            if(c["dynobench"]["model_file"]) require(
                std::filesystem::weakly_canonical(file.parent_path()/c["dynobench"]["model_file"].as<std::string>())==
                std::filesystem::weakly_canonical(anytime_file.parent_path()/anytime["dynobench"]["model_file"].as<std::string>()),
                "anytime uses unchanged physical model");
            if (system == "unicycle") {
                require(c["dynobench"]["model_file"].IsDefined(), "paper unicycle override");
                const auto m = YAML::LoadFile(
                    (file.parent_path() / c["dynobench"]["model_file"].as<std::string>()).string());
                close(m["max_vel"].as<double>(), 1, 0, "unicycle speed");
                close(m["max_angular_vel"].as<double>(), 1.5, 0, "unicycle turn rate");
            } else if (system == "quad3d") {
                require(c["dynobench"]["model_file"].IsDefined(), "paper quad3d override");
                require(std::isinf(c["df_limits"]["max_acceleration"].as<double>()) &&
                            std::isinf(c["df_limits"]["max_angular_acceleration"].as<double>()),
                        "unpublished extra acceleration caps explicitly disabled");
                const auto m = YAML::LoadFile(
                    (file.parent_path() / c["dynobench"]["model_file"].as<std::string>()).string());
                close(m["m"].as<double>(), system == "quad3d" ? 1 : .034, 1e-12, "paper mass");
                require(!m["motor_control"].as<bool>(), "wrench mode");
                close(m["J_v"][2].as<double>(), .2, 1e-12, "yaw inertia");
                close(m["u_ub"][1].as<double>() * 20, 2, 1e-12, "torque normalization");
            } else {
                require(!c["dynobench"]["model_file"].IsDefined(),
                        "quad2d uses DynoBench source model");
                require(!c["dynobench"]["models_directory"].IsDefined(),
                        "models_directory removed");
            }
        }
    });
}
