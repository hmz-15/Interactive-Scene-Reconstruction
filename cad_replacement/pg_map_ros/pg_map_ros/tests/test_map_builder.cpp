#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cassert>

#include "pg_map_ros/nodes.h"
#include "pg_map_ros/parse_graph_map.h"

using namespace std;
using namespace pgm;


void test_create_map()
{
    cout << "* Test Create Map *" << endl;

    ObjectNode ob_node(0, "piggy");
    ParseGraphMap pg1(ob_node);

    ConceptNode cc_node(1, "happiness");
    ParseGraphMap pg2(cc_node);

    ObjectNode::Ptr ob_ptr = make_shared<ObjectNode>(2, "doggy");
    ParseGraphMap pg3(ob_ptr);


    cout << ob_node << endl;
    cout << pg1.getRoot() << endl;

    cout << cc_node << endl;
    cout << pg2.getRoot() << endl;

    cout << ob_ptr << ": " << *ob_ptr << endl;
    cout << pg3.getRoot() << endl;

    cout << "---------------------------------" << endl;
}


void test_build_flat_tree()
{
    cout << "* Test Build Flat Tree *" << endl;
    
    ParseGraphMap pg_map;

    cout << pg_map.getRoot() << endl;

    // add a piggy
    ObjectNode obj_1(1, "piggy");
    pg_map.push_back(obj_1);

    cout << "Add piggy" << endl;
    
    ObjectNode obj_2(2, "doggy");
    pg_map.push_back(obj_2);

    ObjectNode obj_3(3, "kitty");
    pg_map.push_back(obj_3);

    cout << pg_map.getRoot() << endl;


    ConceptNode cc_4(4, "lab");
    pg_map.push_back(cc_4);

    ConceptNode::Ptr root = dynamic_pointer_cast<ConceptNode>(pg_map.getRoot());
    
    root->setConcept("Animal");
    cout << pg_map.getRoot() << endl;
    cout << pg_map.getRoot()->getNodeType() << endl;

    cout << "---------------------------------" << endl;
}


void test_map_structure()
{
    cout << "* Test Map Structure *" << endl;

    ParseGraphMap pg(ConceptNode(0, "Room"));
    
    ObjectNode o1(1, "Robot");
    pg.push_back(o1);

    ConceptNode c1(2, "Workspace");
    c1.addChild( make_shared<ObjectNode>(3, "Chair") );
    c1.addChild( make_shared<ObjectNode>(4, "Desk") );
    pg.push_back(c1);

    ObjectNode o4(5, "Fridge");
    pg.push_back(o4);

    ObjectNode o5(6, "Coke");
    pg.insertNode(5, o5);

    vector<PgEdge> edges = pg.getEdges();
    cout << "PG Edges" << endl;
    for(auto e : edges)
    {
        cout << e << endl;
    }

    vector<NodeBase::Ptr> nodes = pg.getNodes();
    cout << "PG Nodes" << endl;
    for(auto n : nodes)
        cout << n << endl;

    string pgstr_out = pg.dump();
    cout << "pg_json = " << pgstr_out << endl;
    
    cout << "Save graph to file: `pg_sample.json`" << endl;
    pg.save("pg_sample.json");

    ifstream fin("pg_sample.json");
    if(fin.is_open())
    {
        // read JSON file into a string
        stringstream str_stream;
        str_stream << fin.rdbuf();
        string pgstr_in = str_stream.str();
        fin.close();

        cout << "pg_json from file: " << pgstr_in << endl;
        assert(pgstr_out == pgstr_in);
    }
    else
    {
        cout << "[Failed] Cannot open file `pg_sample.json`" << endl;
        assert(false);
    }
    
    system("rm -f pg_sample.json");
    cout << "File `pg_sample.json` deleted" << endl;
    
    cout << "---------------------------------" << endl;
}


void test_delete_node()
{
    cout << "* Test Delete Node *" << endl;

    ParseGraphMap pg(ConceptNode(0, "Room"));
    
    ObjectNode o1(1, "Robot");
    pg.push_back(o1);

    ConceptNode c1(2, "Workspace");
    c1.addChild( make_shared<ObjectNode>(3, "Chair") );
    c1.addChild( make_shared<ObjectNode>(4, "Desk") );
    pg.push_back(c1);

    ObjectNode o4(5, "Fridge");
    pg.push_back(o4);

    ObjectNode o5(6, "Coke");
    pg.insertNode(5, o5);

    vector<PgEdge> edges = pg.getEdges();
    cout << "PG Edges" << endl;
    for(auto e : edges)
    {
        cout << e << endl;
    }

    vector<NodeBase::Ptr> nodes = pg.getNodes();
    cout << "PG Nodes: " << nodes.size() << endl;
    
    // delete worksapce
    assert(pg.deleteNode(2) == 3);

    edges = pg.getEdges();
    cout << "PG Edges" << endl;
    for(auto e : edges)
    {
        cout << e << endl;
    }

    nodes = pg.getNodes();
    cout << "PG Nodes: " << nodes.size() << endl;


    // delete fridge
    assert(pg.deleteNode(5) == 2);

    edges = pg.getEdges();
    cout << "PG Edges" << endl;
    for(auto e : edges)
    {
        cout << e << endl;
    }

    nodes = pg.getNodes();
    cout << "PG Nodes: " << nodes.size() << endl;

    assert(pg.validate() == true);
    cout << "Parse graph is valid" << endl;

    cout << "---------------------------------" << endl;
}


void test_move_node()
{
    cout << "* Test Move Node *" << endl;

    ParseGraphMap pg(ConceptNode(0, "Room"));
    
    ObjectNode o1(1, "Robot");
    pg.push_back(o1);

    ConceptNode c1(2, "Workspace");
    c1.addChild( make_shared<ObjectNode>(3, "Chair") );
    c1.addChild( make_shared<ObjectNode>(4, "Desk") );
    pg.push_back(c1);

    ObjectNode o4(5, "Fridge");
    pg.push_back(o4);

    ObjectNode o5(6, "Coke");
    pg.push_back(o5);

    vector<PgEdge> edges = pg.getEdges();
    cout << "PG Edges" << endl;
    for(auto e : edges)
        cout << e << endl;

    // move coke under fridge
    NodeBase::Ptr ptr = pg.moveNode(6, 5);
    assert(ptr->getID() == 5);

    edges = pg.getEdges();
    cout << "PG Edges" << endl;
    for(auto e : edges)
        cout << e << endl;


    assert(pg.validate() == true);
    cout << "Parse graph is valid" << endl;

    cout << "---------------------------------" << endl;
}


void test_merge_node()
{
    cout << "* Test Merge Node *" << endl;

    ParseGraphMap pg(ConceptNode(0, "A"));
    
    pg.push_back(ObjectNode(1, "B"));
    pg.push_back(ObjectNode(2, "K"));
    pg.push_back(ObjectNode(3, "G"));
    pg.push_back(ObjectNode(4, "E"));
    pg.push_back(ObjectNode(5, "J"));
    pg.push_back(ObjectNode(8, "D"));

    vector<PgEdge> edges = pg.getEdges();
    cout << "PG Edges" << endl;
    for(auto e : edges)
        cout << e << endl;

    pg.mergeNodes({2, 3, 5}, ConceptNode(6, "C"));

    edges = pg.getEdges();
    cout << "PG Edges" << endl;
    for(auto e : edges)
        cout << e << endl;

    pg.mergeNodes({2, 5}, ConceptNode(7, "F"));

    edges = pg.getEdges();
    cout << "PG Edges" << endl;
    for(auto e : edges)
        cout << e << endl;

    assert(pg.validate() == true);
    cout << "Parse graph is valid" << endl;

    cout << "---------------------------------" << endl;
}

int main()
{
    test_create_map();
    test_build_flat_tree();
    test_map_structure();
    test_delete_node();
    test_move_node();
    test_merge_node();

    return 0;
}