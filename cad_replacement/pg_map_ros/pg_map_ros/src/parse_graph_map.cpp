#include "pg_map_ros/parse_graph_map.h"

using namespace std;
using json = nlohmann::json;

namespace pgm
{

ParseGraphMap::ParseGraphMap()
{
    setRoot_(make_shared<ConceptNode>(0, "root"));
}


ParseGraphMap::ParseGraphMap(NodeBase::Ptr root)
{
    setRoot_(root);
}


ParseGraphMap::ParseGraphMap(const ObjectNode &root)
{
    setRoot_(make_shared<ObjectNode>(root));
}


ParseGraphMap::ParseGraphMap(const ConceptNode &root)
{
    setRoot_(make_shared<ConceptNode>(root));
}


/*******************************************************************
 * Modifiers
 ******************************************************************/
/**
 * Insert a concept node under the root
 * 
 * @param child the child node to be inserted
 * @return a base-class pointer of the inserted object node
 */
NodeBase::Ptr ParseGraphMap::push_back(const ObjectNode &obj_node)
{
    return insertNode(root_, obj_node);
}


/**
 * Insert a concept node under the root
 * 
 * @param child the child node to be inserted
 * @return a base-class pointer of the inserted concept node
 */
NodeBase::Ptr ParseGraphMap::push_back(const ConceptNode &cc_node)
{
    return insertNode(root_, cc_node);
}


/**
 * Insert a concept node under parent node
 * 
 * @param parent_id the ID of the parent node
 * @param child the child node to be inserted
 * @return a base-class pointer of the inserted concept node
 */ 
NodeBase::Ptr ParseGraphMap::insertNode(int parent_id, const ConceptNode &child)
{
    // make_shared will copy cc_node to a new instance
    NodeBase::Ptr parent = getNode(parent_id);
    
    // exit with error
    if(parent == nullptr)
    {
        string err_msg = "[ParseGraphMap::insertNode] Parent ID: " + to_string(parent_id) + " does not exist";
        error(ErrorType::NonExistNode, err_msg);
    }
        
    
    return insertNode_(parent, make_shared<ConceptNode>(child));
}


/**
 * Insert a object node under parent node
 * 
 * @param parent_id the ID of the parent node
 * @param child the child node to be inserted
 * @return a base-class pointer of the inserted object node
 */
NodeBase::Ptr ParseGraphMap::insertNode(int parent_id, const ObjectNode &child)
{
    // make_shared will copy cc_node to a new instance
    NodeBase::Ptr parent = getNode(parent_id);

    // exit with error
    if(parent == nullptr)
    {
        string err_msg = "[ParseGraphMap::insertNode] Parent ID: " + to_string(parent_id) + " does not exist";
        error(ErrorType::NonExistNode, err_msg);
    }

    return insertNode_(parent, make_shared<ObjectNode>(child));
}


/**
 * Insert a concept node under parent node
 * 
 * @param parent the pointer of the parent node
 * @param child the child node to be inserted
 * @return a base-class pointer of the inserted concept node
 */ 
NodeBase::Ptr ParseGraphMap::insertNode(NodeBase::Ptr parent, const ConceptNode &child)
{  
    // exit with error
    if( !isNodeExist_(parent) )
    {
        string err_msg = "[ParseGraphMap::insertNode] Parent pointer does not exist in the parse graph";
        error(ErrorType::NonExistNode, err_msg);
    }

    // make_shared will copy cc_node to a new instance
    return insertNode_(parent, make_shared<ConceptNode>(child));
}


/**
 * Insert a object node under parent node
 * 
 * @param parent the pointer of the parent node
 * @param child the child node to be inserted
 * @return a base-class pointer of the inserted object node
 */ 
NodeBase::Ptr ParseGraphMap::insertNode(NodeBase::Ptr parent, const ObjectNode &child)
{
    // exit with error
    if( !isNodeExist_(parent) )
    {
        string err_msg = "[ParseGraphMap::insertNode] Parent pointer does not exist in the parse graph";
        error(ErrorType::NonExistNode, err_msg);
    }

    // make_shared will copy obj_node to a new instance
    return insertNode_(parent, make_shared<ObjectNode>(child));
}


/**
 * Delete a node in the parse graph
 * 
 * @param id the ID of the node to be deleted
 * @return number of nodes removed
 */
int ParseGraphMap::deleteNode(int node_id)
{
    if(node_dict_.find(node_id) == node_dict_.end())
    {
        string err_msg = "[ParseGraphMap::deleteNode] Node ID: " + to_string(node_id)
            + " does not exist in the parse graph";
        error(ErrorType::NonExistNode, err_msg);
    }
    
    return deleteNode(node_dict_[node_id]);
}


/**
 * Delete a node in the parse graph
 * 
 * @param pnode the pointer of the node to be deleted
 * @return number of nodes removed
 */
int ParseGraphMap::deleteNode(NodeBase::Ptr pnode)
{
    // Removing root node is forbidden
    if(pnode == root_)
    {
        string err_msg = "[ParseGraphMap::deleteNode] Root node `ID = " 
            + to_string(pnode->getID()) + "` cannot be deleted";
        error(ErrorType::BadOperation, err_msg);
    }
    
    NodeBase::Ptr parent = findParentNode(pnode);
    
    // update the node dictionary and edge list
    // this operation must be done before deleting the node in pg
    int n_rm_nodes = nodeDictRemove_(pnode);
    edgeSetRemove_(parent, pnode);

    // remove the child in the tree
    parent->deleteChild(pnode);

    return n_rm_nodes;
}


/**
 * Move a node to another position
 * 
 * Move node A under the node B. Node A will be removed from its
 * original parent, and will be added under the node B.
 * 
 * @param pa the pointer of the node A
 * @param pb the pointer of the node B
 * @return the base-class pointer of node B
 */
NodeBase::Ptr ParseGraphMap::moveNode(NodeBase::Ptr pa, NodeBase::Ptr pb)
{
    if(!pa || !pb || !isNodeExist_(pa) || !isNodeExist_(pb))
    {
        string err_msg = "[ParseGraphMap::moveNode] Some of the node does not exists in the parse graph";
        error(ErrorType::NonExistNode, err_msg);
    }
    
    deleteNode(pa);
    insertNode_(pb, pa);

    return pb;
}


/**
 * Move a node to another position
 * 
 * Move node A under the node B. Node A will be removed from its
 * original parent, and will be added under the node B.
 * 
 * @param ida the ID of the node A
 * @param idb the ID of the node B
 * @return the base-class pointer of node B
 */
NodeBase::Ptr ParseGraphMap::moveNode(int ida, int idb)
{
    NodeBase::Ptr pa = getNode(ida);
    NodeBase::Ptr pb = getNode(idb);

    return moveNode(pa, pb);
}


/**
 * Merge a list of nodes into a concept node
 * 
 * Merge a list of nodes into a concept node, the concept node
 * will be added directly under the lowest commom ancester of 
 * those merged nodes. 
 * 
 * @param ids a list of IDs of the node to be merged
 * @param cnode the merged ConceptNode object 
 * @return the base-class pointer of the parent of merged nodes
 */
NodeBase::Ptr ParseGraphMap::mergeNodes(const vector<int> &ids, const ConceptNode &cnode)
{
    vector<NodeBase::Ptr> ptrs;
    for(auto &id : ids)
    {
        if( !isNodeExist_(id) )
        {
            string msg = "[ParseGraphMap::mergeNodes] does not have ID: " + to_string(id);
            error(ErrorType::NonExistNode, msg);
        }
        ptrs.emplace_back(node_dict_[id]);
    }
    return mergeNodes(ptrs, cnode);
}


/**
 * Merge a list of nodes into a object node
 * 
 * Merge a list of nodes into a object node, the object node
 * will be added directly under the lowest commom ancester of 
 * those merged nodes. 
 * 
 * @param ids a list of IDs of the node to be merged
 * @param onode the merged ObjectNode object 
 * @return the base-class pointer of the parent of merged nodes
 */
NodeBase::Ptr ParseGraphMap::mergeNodes(const vector<int> &ids, const ObjectNode &onode)
{
    vector<NodeBase::Ptr> ptrs;
    for(auto &id : ids)
    {
        if( !isNodeExist_(id) )
        {
            string msg = "[ParseGraphMap::mergeNodes] does not have ID: " + to_string(id);
            error(ErrorType::NonExistNode, msg);
        }
        ptrs.emplace_back(node_dict_[id]);
    }
    return mergeNodes(ptrs, onode);
}


/**
 * Merge a list of nodes into a concept node
 * 
 * Merge a list of nodes into a concept node, the concept node
 * will be added directly under the lowest commom ancester of 
 * those merged nodes. 
 * 
 * @param ptrs a list of pointers of the node to be merged
 * @param cnode the merged ConceptNode object 
 * @return the base-class pointer of the parent of merged nodes
 */
NodeBase::Ptr ParseGraphMap::mergeNodes(const vector<NodeBase::Ptr> &ptrs, const ConceptNode & cnode)
{
    // mergeNodes_ will validate if pointers in ptrs exist in the parse graph
    return mergeNodes_(ptrs, make_shared<ConceptNode>(cnode));
}


/**
 * Merge a list of nodes into a object node
 * 
 * Merge a list of nodes into a object node, the object node
 * will be added directly under the lowest commom ancester of 
 * those merged nodes. 
 * 
 * @param ptrs a list of pointers of the node to be merged
 * @param onode the merged ObjectNode object 
 * @return the base-class pointer of the parent of merged nodes
 */
NodeBase::Ptr ParseGraphMap::mergeNodes(const vector<NodeBase::Ptr> &ptrs, const ObjectNode & onode)
{
    // mergeNodes_ will validate if pointers in ptrs exist in the parse graph
    return mergeNodes_(ptrs, make_shared<ObjectNode>(onode));
}


/*******************************************************************
 * Element Access
 ******************************************************************/
/**
 * Validate current parse graph
 * 
 * Check if node_dict edge_map are correct. This method could
 * be time-consuming.
 * 
 * @return true if valid, otherwise false
 */
bool ParseGraphMap::validate()
{
    return (validateNodes_() && validateEdges_());
}


/**
 * Get the pointer of the root node of the parse graph
 * 
 * @return the pointer of the root node
 */
NodeBase::Ptr ParseGraphMap::getRoot()
{
    return root_;
}


/**
 * Get a node pointer according to its ID
 * 
 * @param id the ID the node
 * @return the pointer of the node, a `nullptr` is return
 *      if ID does not exist
 */
NodeBase::Ptr ParseGraphMap::getNode(int id)
{
    if(node_dict_.find(id) == node_dict_.end())
        return nullptr;
    
    return node_dict_[id];
}


/**
 * Find the direct parent of the given node
 * 
 * @param pnode the pointer of the node looks for parent
 * @return parent node pointer
 */
NodeBase::Ptr ParseGraphMap::findParentNode(NodeBase::Ptr pnode)
{
    return findParentNode_(root_, pnode);
}

/**
 * Find the direct parent of the given node
 * 
 * @param id the ID of the node that looks for parent
 * @return parent node pointer
 */
NodeBase::Ptr ParseGraphMap::findParentNode(int id)
{
    if(node_dict_.find(id) == node_dict_.end())
        return nullptr;
    
    return findParentNode(node_dict_[id]);
}


/**
 * Get all edges in the parse graph
 * 
 * Edge is define by a tuple (src_id, dst_id), edge from 
 * the source ID to the destination ID.
 * 
 * @return A vector of edge tuples 
 */
vector<PgEdge> ParseGraphMap::getEdges()
{
    vector<PgEdge> edge_list;

    for(const auto &it : edge_map_)
        edge_list.push_back(it.second);

    return edge_list;
}


/**
 * Get a all node pointers in the parse graph
 * 
 * @return A vector of pointers of nodes in the parse graph
 */
vector<NodeBase::Ptr> ParseGraphMap::getNodes()
{
    vector<NodeBase::Ptr> nodes;

    for(auto &it : node_dict_)
        nodes.push_back(it.second);

    return nodes;
}


/**
 * Dump the parse graph into a JSON string
 * 
 * @return A JSON string contain parse graph information
 *      {
 *          "edges": [(0, 1), (0, 2), (1, 3), ...],
 *          "root_id": 0,
 *          "nodes": [
 *              {<JSON-for-node>}, {<JSON-for-node>}, ...
 *          ]
 *      }
 */
std::string ParseGraphMap::dump()
{
    json pg_json;
    
    pg_json["root_id"] = root_->getID();

    vector<json> edge_info;
    for(auto &it : edge_map_)
    {
        edge_info.push_back(generateEdgeJson_(it.second));
    }
    pg_json["edges"] = edge_info;
    

    // generate node info json
    vector<json> node_info;
    for(auto &it : node_dict_)
    {
        node_info.push_back(generateNodeJson_(it.second));
    }
    pg_json["nodes"] = node_info;

    return pg_json.dump(4);
}


/**
 * Save the parse graph to a file
 * 
 * @param filename the filename (string) of the output file
 * @return error code (int), 0 if succeed, otherwise error prompted
 */
int ParseGraphMap::save(const string filename)
{
    string pg_jsonstr = dump();

    ofstream fout(filename);
  
    if( fout.is_open() )
    {
        fout << pg_jsonstr;
        fout.close();
    }
    else
    {
        cout << "[Warning] Cannot open file: " << filename << endl;
        return (int)ErrorType::FailOpenFile;
    }

    return 0;
}


/*******************************************************************
 * Internal Functions
 ******************************************************************/
/**
 * [Internal] Validate if current nodes match node_dict_
 * 
 * 
 * @return true if valid, otherwise false
 */
bool ParseGraphMap::validateNodes_()
{
    vector<NodeBase::Ptr> all_nodes = traverseNode_(root_);
    
    for(const auto &it : all_nodes)
    {
        if(node_dict_.find(it->getID()) == node_dict_.end())
            return false;
        // check if pointer addr. match the one stored in the node_dict
        if(node_dict_[it->getID()] != it)
            return false;
    }

    return true;
}


/**
 * [Internal] Validate if current edges match edge_map
 * 
 * @return true if valid, otherwise false
 */
bool ParseGraphMap::validateEdges_()
{
    vector<vector<int> > all_edges = traverseEdge_(root_);

    for(const auto &it : all_edges)
    {
        if(edge_map_.find(make_pair(it[0], it[1])) == edge_map_.end())
            return false;
    }

    return true;
}

/**
 * [Internal] Check whether a node exists or not
 * 
 * @param id the id of node to be checked
 * @return true if exists, otherwise false
 */
bool ParseGraphMap::isNodeExist_(int id)
{
    return (node_dict_.find(id) != node_dict_.end());
}


/**
 * [Internal] Check whether a node exists or not
 * 
 * @param pnode the pointer of node to be checked
 * @return true if exists, otherwise false
 */
bool ParseGraphMap::isNodeExist_(NodeBase::Ptr pnode)
{
    return isNodeExist_( pnode->getID() );
}


/**
 * [Internal] Insert a node under parent node
 * 
 * @param parent the pointer of the parent node
 * @param child the pointer of the child node to be inserted
 * @return a base-class pointer of the inserted node
 */
NodeBase::Ptr ParseGraphMap::insertNode_(NodeBase::Ptr parent, NodeBase::Ptr child)
{
    parent->addChild(child);

    nodeDictAdd_(child);
    edgeSetAdd_(parent, child);

    return child;
}


/**
 * [Internal] Add edges in the edge_map_
 * 
 * Add edge between parent and child to the edge_map_, add all 
 * descedants edges of child node to the edge_map as well.
 * 
 * @param parent the parent node
 * @param child the child node
 * @return number of edges added
 */
size_t ParseGraphMap::edgeSetAdd_(const NodeBase::Ptr parent, const NodeBase::Ptr child)
{
    vector<vector<int> > subtree_edges = traverseEdge_(child);

    edge_map_.insert({make_pair(parent->getID(), child->getID()), PgEdge(parent->getID(), child->getID())});
    
    for(const auto &e : subtree_edges)
    {
        edge_map_.insert({make_pair(e[0], e[1]), PgEdge(e[0], e[1])});
    }

    return subtree_edges.size() + 1;
}


/**
 * Remove edges in the edge_map_
 * 
 * The edge between parent and child, and all descendants edges of child
 * will be removed
 * 
 * @param parent the parent node
 * @param child the child node
 * @return number of edges removed
 */
size_t ParseGraphMap::edgeSetRemove_(const NodeBase::Ptr parent, const NodeBase::Ptr child)
{
    vector<vector<int> > rm_edges = traverseEdge_(child);

    edge_map_.erase( make_pair(parent->getID(), child->getID()) );
    for(const auto &it : rm_edges)
    {
        edge_map_.erase( make_pair(it[0], it[1]) );
    }

    return rm_edges.size() + 1;
}


/**
 * [Internal] Add node to the node_dict_
 * 
 * Add node and all its descedants to the node_dict_.
 * 
 * @param pnode the parent node
 * @return number of nodes added
 */
size_t ParseGraphMap::nodeDictAdd_(NodeBase::Ptr pnode)
{   
    vector<NodeBase::Ptr> nodeset = traverseNode_(pnode);

    for(const auto &it : nodeset)
    {
        // check duplicated ID
        if( isNodeExist_(it->getID()) )
        {
            string err_msg = "[ParseGraphMap::nodeDictAdd_] duplicated node ID: "
                + to_string(it->getID());
            error(ErrorType::DuplicatedNodeID, err_msg);
        }
        node_dict_[it->getID()] = it;
    }

    return nodeset.size();
}


/**
 * Remove a node and all its descendants in the node_dict_
 * 
 * The node and all its descendants will be removed
 * 
 * @param pnode the node to be removed
 * @return number of nodes removed
 */ 
size_t ParseGraphMap::nodeDictRemove_(NodeBase::Ptr pnode)
{
    vector<NodeBase::Ptr> rm_nodes = traverseNode_(pnode);
    for(auto &ptr : rm_nodes)
    {
        node_dict_.erase( ptr->getID() );
    }
    return rm_nodes.size();
}


/**
 * [Internal] Set root node
 * 
 * Set the root node of the parse graph, and rebuild the node_dict
 * and the edge_map
 * 
 * @param root the pointer of the root node
 */
void ParseGraphMap::setRoot_(NodeBase::Ptr root)
{
    root_ = root;

    node_dict_.clear();
    edge_map_.clear();
    nodeDictAdd_(root);
}


/**
 * [Internal] Generate JSON for edge information
 * 
 * @param edge PgEdge object of the edge
 * @return JSON object for ConceptNode/ObjectNode
 */
json ParseGraphMap::generateEdgeJson_(const PgEdge &edge)
{
    json ej;

    ej["src_id"] = edge.getSrcID();
    ej["dst_id"] = edge.getDstID();

    return ej;
}

/**
 * [Internal] Generate JSON for node information
 * 
 * @param pnode the pointer of the target node 
 * @return JSON object for ConceptNode/ObjectNode
 */
json ParseGraphMap::generateNodeJson_(NodeBase::Ptr pnode)
{
    if(pnode->getNodeType() == NodeType::ConceptNode)
        return generateConceptNodeJson_(dynamic_pointer_cast<ConceptNode>(pnode));
    else if(pnode->getNodeType() == NodeType::ObjectNode)
        return generateObjectNodeJson_(dynamic_pointer_cast<ObjectNode>(pnode));
    else
    {
        cerr << "[ParseGraphMap::generateNodeJson_]: Unknown NodeType" << pnode->getNodeType() << endl;
        return {};
    }
}


/**
 * [Internal] Generate JSON for ConceptNode information
 * 
 * @param pnode the pointer of the target node 
 * @return JSON object
 *      {
 *          "node_type": "ConceptNode",
 *          "id": 123,
 *          "concept": "workspace"
 *      }
 */
json ParseGraphMap::generateConceptNodeJson_(ConceptNode::Ptr pnode)
{
    json nj;
    
    nj["node_type"] = "ConceptNode";
    nj["id"] = pnode->getID();
    nj["concept"] = pnode->getConcept();

    return nj;
}


/**
 * [Internal] Generate JSON for ObjectNode information
 * 
 * @param pnode the pointer of the target node 
 * @return JSON object
 *      {
 *          "node_type": "ObjectNode",
 *          "id": 123,
 *          "label": "desk",
 *          "position": [0, 0, 0], // [x, y, z]
 *          "orientation": [0, 0, 0, 1], // [x, y, z, w]
 *          "box": [1, 1, 1], // [length, width, height]
 *      }
 */
json ParseGraphMap::generateObjectNodeJson_(ObjectNode::Ptr pnode)
{
    json nj;

    nj["node_type"] = "ObjectNode";
    nj["id"] = pnode->getID();
    nj["cad_id"] = pnode->getCadID();
    nj["scale"] = pnode->getScale();
    nj["label"] = pnode->getLabel();
    nj["position"] = pnode->getPosition().vectorize();
    nj["orientation"] = pnode->getOrientation().vectorize();
    nj["box"] = pnode->getBBox().vectorize();
    nj["ious"] = pnode->getIoUs();

    return nj;
}


/**
 * Traverse all nodes started from pnode
 * 
 * @param pnode the root node where the traverse starts
 * @return a vector of NodeBase::Ptr that resides under pnode
 */ 
vector<NodeBase::Ptr> ParseGraphMap::traverseNode_(NodeBase::Ptr pnode)
{
    vector<NodeBase::Ptr> nodes;

    nodes.push_back(pnode);

    for(auto &it : pnode->getChildren())
    {
        vector<NodeBase::Ptr> subtree_nodes = traverseNode_(it);
        move(subtree_nodes.begin(), subtree_nodes.end(), back_inserter(nodes));
    }

    return nodes;
}


/**
 * [Internal] Traverse all edges started from pnode
 * 
 * @param pnode the root node where the traverse starts
 * @return a vector of tuple [(src, dst), ...] that contains all
 *     edges resides under the pnode
 */ 
vector<vector<int> > ParseGraphMap::traverseEdge_(NodeBase::Ptr pnode)
{
    vector<vector<int> > edges;

    for(auto &it : pnode->getChildren())
    {
        vector<vector<int> > child_edges = traverseEdge_(it);
        move(child_edges.begin(), child_edges.end(), back_inserter(edges));
        edges.push_back( vector<int>({pnode->getID(), it->getID()}) );
    }

    return edges;
}


/**
 * Find the parent of the target node given the root
 * 
 * @param pnode the pointer of the target node that looks 
 *     for parent node
 * @param root the root the search tree
 * @return the parent node pointer of the target node
 */ 
NodeBase::Ptr ParseGraphMap::findParentNode_(NodeBase::Ptr root, NodeBase::Ptr pnode)
{
    if(root == pnode)
        return nullptr;
    
    vector<NodeBase::Ptr> children = root->getChildren();

    // root is the parent of the pnode
    if(find(children.begin(), children.end(), pnode) != children.end())
        return root;
    
    // DFS all children
    for(auto it : children)
    {
        NodeBase::Ptr parent = findParentNode_(it, pnode);
        if(parent != nullptr)
            return parent;
    }

    return nullptr;
}


/**
 * [Internal] Merge a list of nodes into one node
 * 
 * Merge a list of nodes into one node, the newly common parent
 * node will be added directly under the lowest commom ancester of 
 * those nodes to be merged. 
 * 
 * @param ptrs a list of pointers of the node to be merged
 * @param pnode the pointer of new common parent node 
 * @return the base-class pointer of the parent of merged nodes
 */
NodeBase::Ptr ParseGraphMap::mergeNodes_(const vector<NodeBase::Ptr> &ptrs, NodeBase::Ptr pnode)
{
    if(ptrs.size() == 0)
        error(ErrorType::NullError, "[ParseGraphMap::mergeNodes_]: empty pointer list");        
    
    // check if all pointers exist in the parse graph
    for(auto &it : ptrs)
    {
        if( !isNodeExist_(it) )
        {
            string msg = "[ParseGraphMap::mergeNodes_] node ID: " 
                + to_string(it->getID()) + " does not exist in the parse graph";
            error(ErrorType::NonExistNode, msg);
        }
    }

    // find the lowest common ancester of nodes in ptrs
    NodeBase::Ptr lca = findLCA_(ptrs);

    // if lca is one of the pointer in ptrs, then find its parent node
    if(find(ptrs.begin(), ptrs.end(), lca) != ptrs.end())
        lca = findParentNode(lca);

    // insert the merged node under the `lca`
    insertNode_(lca, pnode);
    // move all nodes in `ptrs` under the merged node `pnode`
    for(auto &it : ptrs)
        moveNode(it, pnode);

    return pnode;
}


/**
 * [Internal] Find the Lowest Common Ancester (LCA)
 * 
 * Find the lowest common ancester of the given nodes 
 * 
 * @param ptrs a list of pointers of the node to be merged
 * @return the pointer of the lowest common ancester
 */
NodeBase::Ptr ParseGraphMap::findLCA_(const vector<NodeBase::Ptr> &ptrs)
{
    NodeBase::Ptr lca = ptrs[0];

    // recursively find the lowest common ancester from pair to pair
    // LCA(pa, pb ,pc) = LCA(LCA(pa, pb), pc)
    for(size_t i = 1; i < ptrs.size(); i++)
        lca = lcaHelper_(root_, lca, ptrs[i]);
    
    return lca;
}


/**
 * [Internal] Find the LCA between a pair of nodes
 * 
 * Find the lowest common ancester of a pair of nodes
 * 
 * @param root the root node to search from
 * @param pa the pointer of one node in the pair
 * @param pb the pointer of one node in the pair
 * @return the pointer of the lowest common ancester
 */
NodeBase::Ptr ParseGraphMap::lcaHelper_(NodeBase::Ptr root, NodeBase::Ptr pa, NodeBase::Ptr pb)
{
    if(!root || root == pa || root == pb)
        return root;

    vector<NodeBase::Ptr> children(root->getChildren());
    vector<NodeBase::Ptr> non_nullptrs;

    for(auto &it : children)
    {
        NodeBase::Ptr ret = lcaHelper_(it, pa, pb);
        if(ret)
            non_nullptrs.push_back(ret);
    }

    // if there are two non-null pointers, then root must be the LCA of pa and pb
    if(non_nullptrs.size() == 2)
        return root;
    // if there is only one non-null pointer, then return the this non-null pointer
    // this indicates that pa or pb is under this subtree
    if(non_nullptrs.size() == 1)
        return non_nullptrs[0];
    // if there are all null pointers, then return a nullptr
    // this indicates that none of pa nor pb is under this subtree
    return nullptr;
}

} // end of namespace pgm

