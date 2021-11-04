#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <tuple>
#include <cassert>

#include "pg_map_ros/nodes.h"
#include "pg_map_ros/parse_graph_map.h"


using namespace std;
using namespace pgm;


ParseGraphMap::Ptr generatePG(const char *scene_structure_input){
    // ParseGraphMap::Ptr pg_ptr = new()
    ifstream fin(scene_structure_input);

    if( !fin.is_open() ){
        cout << "[ERROR] Cannot open file: " << scene_structure_input << endl;
        exit(1);
    }

    int id = -1;
    int pid = -1;
    string label;

    fin >> id >> label;
    cout << "Add [" << id << ", " << label << "]" << endl;

    ParseGraphMap::Ptr pg_ptr = make_shared<ParseGraphMap>(ParseGraphMap(ConceptNode(id, label)));

    while(fin >> id >> label >> pid){
        ObjectNode node(id, label);
        node.setIoUs(VecPair<int, float>({{id, 1}}));

        pg_ptr->insertNode(pid, node);
        cout << "Add [" << id << ", " << label << ", " << pid << "]" << endl;
    }

    return pg_ptr;
}


ParseGraphMap::Ptr generatePG(const string &scene_structure_input){
    return generatePG(scene_structure_input.c_str());
}


int main(int argc, char **argv)
{
    if(argc < 3){
        cout << "Usage:\n";
        cout << "\t./annotate_cg <scene-structure-txt-dir> <output-json-dir>" << endl;
        return 1;
    }

    vector<string> vec = {
        "225",
        "231",
        "249",
        "322"
    };

    for(auto &idx : vec){
        string file_in = string(argv[1]) + "/" + idx + "_structure.txt";
        string file_out = string(argv[2]) + "/" + idx + "_annotated.json";

        
        cout << "Processing: " << file_in << endl;

        ParseGraphMap::Ptr pg_ptr = generatePG(file_in);
        pg_ptr->save(file_out);

        cout << "Contact Graph JSON was saved at: " << file_out << endl;
    }

    return 0;
}
