#include "ros/ros.h"
#include <std_msgs/String.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <iostream>

const int ROWS = 12;
const int COLS = 12;
enum Cell { EMPTY = 0, WALL = 1, FIRE = 2 };

std::vector<std::vector<int>> grid(ROWS, std::vector<int>(COLS, EMPTY));
bool received = false;

void loadGridFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Could not open map file: %s", filename.c_str());
        return;
    }

    std::string line;
    int row = 0;
    while (std::getline(file, line) && row < ROWS) {
        std::stringstream ss(line);
        std::string val;
        int col = 0;
        while (std::getline(ss, val, ',') && col < COLS) {
            grid[row][col++] = std::stoi(val);
        }
        row++;
    }
    file.close();
    received = true;
    ROS_INFO("Loaded map from full_explored_map.txt");
}

void writeCSP(const std::vector<std::vector<int>>& grid, const std::string& filename) {
    std::ofstream out(filename);
    if (!out.is_open()) {
        ROS_ERROR("Failed to open CSP output file.");
        return;
    }
    out << "// Auto-generated CSP file from ROS map_node\n\n";
    out << "#define ROWS " << grid.size() << ";\n";
    out << "#define COLS " << grid[0].size() << ";\n\n";
    out << "#define E 0;\n#define W 1;\n#define F 2;\n\n";
    out << "var x : {0..ROWS-1} = 11;\n";
    out << "var y : {0..COLS-1} = 6;\n\n";
    out << "var water : {0..2} = 2;\n";
    out << "var world[ROWS][COLS] = [\n";
    for (int i = 0; i < grid.size(); ++i) {
        for (int j = 0; j < grid[i].size(); ++j) {
            out << grid[i][j];
            if (i != grid.size() - 1 || j != grid[i].size() - 1)
                out << ",";
        }
        out << "   // row " << i << "\n ";
    }
    out << "];\n\n";
    out << "Game = \n";
    out << "  [x > 0 && world[x-1][y] != W] MoveUp\n";
    out << "[] [x < ROWS-1 && world[x+1][y] != W] MoveDown\n";
    out << "[] [y > 0 && world[x][y-1] != W] MoveLeft\n";
    out << "[] [y < COLS-1 && world[x][y+1] != W] MoveRight\n";
    out << "[] [world[x][y] == F && water >= 1] Extinguish\n";
    out << "[] [x == 11 && y == 6 && water < 2] Refill;\n\n";
    out << "MoveUp       = move_up    {x = x - 1;}       -> Game;\n";
    out << "MoveDown     = move_down  {x = x + 1;}       -> Game;\n";
    out << "MoveLeft     = move_left  {y = y - 1;}       -> Game;\n";
    out << "MoveRight    = move_right {y = y + 1;}       -> Game;\n";
    out << "Extinguish   = extinguish {world[x][y] = E; water = water - 1;} -> Game;\n";
    out << "Refill \t = refill {water = 2;} -> Game;\n\n";
    out << "System = Game;\n\n";
    out << "#define goalAllFiresExtinguishedAndReturn \n";
    out << "  (&& i:{0..ROWS-1} @ (&& j:{0..COLS-1} @ (world[i][j] != F))) \n";
    out << "  && (x == 11 && y == 6);\n\n";
    out << "#assert Game reaches goalAllFiresExtinguishedAndReturn;\n";
    out.close();
    ROS_INFO("CSP model successfully written.");
}

void runPATAndWritePlanTxt() {
    std::string command = "mono //home/admin/Downloads/PAT3/PAT3.Console.exe -engine 1 "
                          "-csp /home/admin/catkin_ws/src/Assigment_2/firebot_system/world.csp "
                          "/home/admin/catkin_ws/src/Assigment_2/firebot_system/pat_raw_output.txt";
    int result = system(command.c_str());
    if (result != 0)
        ROS_ERROR("Failed to run PAT.");
    else
        ROS_INFO("PAT executed and raw output saved.");
}

std::vector<std::string> splitTrace(const std::string& trace, const std::string& delimiter) {
    std::vector<std::string> moves;
    size_t start = 0;
    size_t end = trace.find(delimiter);

    while (end != std::string::npos) {
        moves.push_back(trace.substr(start, end - start));
        start = end + delimiter.length();
        end = trace.find(delimiter, start);
    }
    moves.push_back(trace.substr(start));
    return moves;
}

size_t findValidEnd(const std::string& line, size_t start) {
    size_t end = line.find('>', start);
    while (end != std::string::npos) {
        if (end == 0 || line[end - 1] != '-') {
            return end;
        }
        end = line.find('>', end + 1);
    }
    return std::string::npos;
}

void trimRawOutput() {
    std::ifstream input("/home/admin/catkin_ws/src/Assigment_2/firebot_system/pat_raw_output.txt");
    std::ofstream output("/home/admin/catkin_ws/src/Assigment_2/firebot_system/robot_movements.txt");

    if (!input.is_open()) {
        ROS_ERROR("Failed to open PAT raw output file for reading.");
        return;
    }

    if (!output.is_open()) {
        ROS_ERROR("Failed to open robot movements file for writing.");
        return;
    }

    std::string line;
    bool found_trace = false;

    while (std::getline(input, line)) {
        size_t start = line.find('<');
        size_t end = findValidEnd(line, start);

        if (start != std::string::npos && end != std::string::npos) {
            std::string trace = line.substr(start + 1, end - start - 1);
            const std::string prefix = "init -> ";

            if (trace.find(prefix) == 0) {
                trace = trace.substr(prefix.length());
            }

            std::vector<std::string> moves = splitTrace(trace, " -> ");

            // Prepend extra move_up for alignment fix
            output << "move_up" << std::endl;

            for (const std::string& move : moves) {
                output << move << std::endl;
            }

            found_trace = true;
            break;
        }
    }

    input.close();
    output.close();

    if (found_trace) {
        ROS_INFO("Cleaned moves saved in robot_movements.txt");
    } else {
        ROS_WARN("No valid trace found in PAT output");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_and_trim_node");
    ros::NodeHandle nh;

    loadGridFromFile("/home/admin/catkin_ws/src/Assigment_2/firebot_system/full_explored_map.txt");

    ROS_INFO("Writing CSP from file map...");
    writeCSP(grid, "/home/admin/catkin_ws/src/Assigment_2/firebot_system/world.csp");

    ROS_INFO("Running PAT to generate plan...");
    runPATAndWritePlanTxt();

    ROS_INFO("Trimming PAT output to extract robot movements...");
    trimRawOutput();

    ROS_INFO("Map processing and plan generation complete!");
    return 0;
}
