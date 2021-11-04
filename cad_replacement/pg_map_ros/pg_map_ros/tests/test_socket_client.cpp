#include "pg_map_ros/tcp_client.h"
#include "pg_map_ros/parse_graph_map.h"
#include "pg_map_ros/3rd_party/json.hpp"

#include <iostream>

using namespace pgm;
using namespace std;
using json = nlohmann::json;


void test_send_pg()
{
    cout << "* Test Send PG via Socket *" << endl;

    TcpClient client("0.0.0.0", 12345);
    ParseGraphMap pg(ConceptNode(0, "Room"));
    
    ObjectNode o1(1, "Robot");
    pg.push_back(o1);

    usleep(2e6);
    cout << "Robot detected" << endl;
    client.sendData(pg.dump());

    ObjectNode o2(3, "Chair");
    pg.push_back(o2);

    usleep(2e6);
    cout << "Chair detected" << endl;
    client.sendData(pg.dump());
    
    ObjectNode o3(5, "Desk");
    pg.push_back(o3);

    usleep(2e6);
    cout << "Desk detected" << endl;
    client.sendData(pg.dump());
    

    ObjectNode o4(7, "Fridge");
    NodeBase::Ptr fridge_ptr = pg.push_back(o4);

    usleep(2e6);
    cout << "Fridge detected" << endl;
    client.sendData(pg.dump());

    pg.mergeNodes({5, 3}, ConceptNode(9, "Workspace"));

    usleep(2e6);
    cout << "Workspace detected" << endl;
    client.sendData(pg.dump());

    ObjectNode o5(4, "Coke");
    NodeBase::Ptr coke_ptr = pg.push_back(o5);

    usleep(2e6);
    cout << "Coke detected" << endl;
    client.sendData(pg.dump());

    pg.moveNode(coke_ptr, fridge_ptr);

    usleep(2e6);
    cout << "Coke is placed in the fridge" << endl;
    client.sendData(pg.dump());

    cout << "---------------------------------" << endl;
}


void send_onr_demo_pg()
{
    cout << "* Send ONR Demo PG *" << endl;

    TcpClient client("0.0.0.0", 12345);
    ParseGraphMap pg(ConceptNode(0, "scene"));
    
    ObjectNode o1(1, "layout");
    pg.push_back(o1);

    ObjectNode o2(2, "chair");
    pg.push_back(o2);
    
    ObjectNode o3(3, "monitor");
    pg.push_back(o3);

    ObjectNode o4(4, "fridge");
    pg.push_back(o4);

    ObjectNode o5(5, "human");
    pg.push_back(o5);

    pg.mergeNodes({2, 3}, ConceptNode(6, "workspace"));

    client.sendData(pg.dump());

    cout << "---------------------------------" << endl;
}

int main()
{
    test_send_pg();
    // send_onr_demo_pg();

    return 0;
}