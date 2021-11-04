#include "pg_map_ros/nodes.h"

#include <iostream>
#include <string>


using namespace std;
using namespace pgm;


void test_print(){
    cout << "* Test Print *" << endl;

    pgm::NodeBase::Ptr nb_ptr = nullptr;
    
    // call derived class function through parent class virtual function
    nb_ptr = make_shared<pgm::ObjectNode>(0, "puppy");
    cout << nb_ptr << endl;

    nb_ptr = make_shared<pgm::ConceptNode>(1, "workspace");
    cout << nb_ptr << endl;

    pgm::ObjectNode piggy(2, "piggy");
    cout << piggy << endl;
    cout << make_shared<pgm::ObjectNode>(piggy) << endl;
    

    pgm::ConceptNode lab_node(3, "lab");
    cout << lab_node << endl;
    cout << make_shared<pgm::ConceptNode>(lab_node) << endl;

    cout << "-----------------------" << endl;
}

void test_dynamic_cast(){
    cout << "* Test Dynamic Cast *" << endl;

    pgm::NodeBase::Ptr nb_ptr = nullptr;

    nb_ptr = make_shared<pgm::ObjectNode>(0, "puppy");
    if(nb_ptr->getNodeType() == pgm::NodeType::ObjectNode){
        pgm::ObjectNode::Ptr obj_node = dynamic_pointer_cast<pgm::ObjectNode>(nb_ptr);
        cout << "Hi there, I am a " << obj_node->getLabel() << ", my id is " 
            << obj_node->getID() << endl;
        cout << "Position: " << obj_node->getPosition() << endl;
        cout << "Orientation: " << obj_node->getOrientation() << endl;
    }

    nb_ptr = make_shared<pgm::ConceptNode>(1, "kitchen");
    if(nb_ptr->getNodeType() == pgm::NodeType::ConceptNode){
        pgm::ConceptNode::Ptr obj_node = dynamic_pointer_cast<pgm::ConceptNode>(nb_ptr);
        cout << "Hi there, I am a " << obj_node->getConcept() << ", my id is " 
            << obj_node->getID() << endl;
    }

    cout << "-----------------------" << endl;
}


void test_object_node(){
    cout << "* Test Object Node *" << endl;

    pgm::NodeBase::Ptr nb_ptr = nullptr;
    
    ObjectNode obj(0, "chair");

    obj.setCadID("chair_cad_model_001");
    obj.setBBox(Point(1, 1, 1));
    obj.setPosition(Point(0.5, 0.5, 0));
    obj.setOrientation(Quaternion(0, 0, 0, 1));
    obj.setScale(0.6);

    
    cout << obj << endl;

    obj.setPose(Point(1, 1, 0), Quaternion(0, 0, 0, 1));

    cout << obj << endl;

    cout << "-----------------------" << endl;
}


void test_object_node_ious(){
    cout << "* Test Object Node IoUs *" << endl;

    pgm::NodeBase::Ptr nb_ptr = nullptr;
    
    ObjectNode obj(0, "chair");

    obj.setCadID("chair_cad_model_001");
    obj.setBBox(Point(1, 1, 1));
    obj.setPosition(Point(0.5, 0.5, 0));
    obj.setOrientation(Quaternion(0, 0, 0, 1));
    obj.setScale(0.6);

    // set IoUs
    obj.setIoUs( pgm::VecPair<int, float>({{1, 0.11}, {2, 0.22}}) );
    cout << obj << endl;
    
    // add multiple IoU
    obj.addIoUs( pgm::VecPair<int, float>({{3, 0.33}, {4, 0.44}}) );
    cout << obj << endl;

    // add single IoU via pair
    obj.addIoU( make_pair(5, 0.55) );
    cout << obj << endl;

    // add single IoU via label, iou_score
    obj.addIoU(6, 0.66);
    cout << obj << endl;

    cout << "-----------------------" << endl;
}



int main(){
    test_print();
    test_dynamic_cast();
    test_object_node();
    test_object_node_ious();

    return 0;
}