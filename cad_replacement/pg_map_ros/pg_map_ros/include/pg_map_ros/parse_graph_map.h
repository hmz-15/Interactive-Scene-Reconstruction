#ifndef _PGM_PARSE_GRAPH_MAP_H
#define _PGM_PARSE_GRAPH_MAP_H

#include <string>
#include <memory>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

#include "nodes.h"
#include "pg_edge.h"
#include "pair_hash.h"
#include "pgm_error.h"
#include "3rd_party/json.hpp"

namespace pgm
{

class ParseGraphMap
{
public:
    using Ptr = std::shared_ptr<ParseGraphMap>;

    ParseGraphMap();
    ParseGraphMap(NodeBase::Ptr);
    ParseGraphMap(const ObjectNode &);
    ParseGraphMap(const ConceptNode &);

    ~ParseGraphMap() = default;


/*******************************************************************
 * Modifiers
 ******************************************************************/
public:
    /**
     * Insert a concept node under the root
     * 
     * @param child the child node to be inserted
     * @return a base-class pointer of the added concept node
     */
    NodeBase::Ptr push_back(const ConceptNode &);

    /**
     * Insert a concept node under the root
     * 
     * @param child the child node to be inserted
     * @return a base-class pointer of the added object node
     */
    NodeBase::Ptr push_back(const ObjectNode &);

    /**
     * Insert a concept node under parent node
     * 
     * @param parent_id the ID of the parent node
     * @param child the child node to be inserted
     * @return a base-class pointer of the inserted concept node
     */ 
    NodeBase::Ptr insertNode(int, const ConceptNode &);

    /**
     * Insert a object node under parent node
     * 
     * @param parent_id the ID of the parent node
     * @param child the child node to be inserted
     * @return a base-class pointer of the inserted object node
     */ 
    NodeBase::Ptr insertNode(int, const ObjectNode &);

    /**
     * Insert a concept node under parent node
     * 
     * @param parent the pointer of the parent node
     * @param child the child node to be inserted
     * @return a base-class pointer of the inserted concept node
     */ 
    NodeBase::Ptr insertNode(NodeBase::Ptr, const ConceptNode &);

    /**
     * Insert a object node under parent node
     * 
     * @param parent the pointer of the parent node
     * @param child the child node to be inserted
     * @return a base-class pointer of the inserted object node
     */ 
    NodeBase::Ptr insertNode(NodeBase::Ptr, const ObjectNode &);
    
    /**
     * Delete a node in the parse graph
     * 
     * @param id the ID of the node to be deleted
     * @return number of nodes removed
     */ 
    int deleteNode(int);

    /**
     * Delete a node in the parse graph
     * 
     * @param pnode the pointer of the node to be deleted
     * @return number of nodes removed
     */
    int deleteNode(NodeBase::Ptr);

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
    NodeBase::Ptr moveNode(NodeBase::Ptr, NodeBase::Ptr);

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
    NodeBase::Ptr moveNode(int, int);

    /**
     * Merge a list of nodes into a concept node
     * 
     * Merge a list of nodes into a concept node, the concept node
     * will be added directly under the lowest commom ancester of 
     * those nodes to be merged. 
     * 
     * @param ids a list of IDs of the node to be merged
     * @param cnode the merged ConceptNode object 
     * @return the base-class pointer of the parent of merged nodes
     */
    NodeBase::Ptr mergeNodes(const std::vector<int> &, const ConceptNode &);

    /**
     * Merge a list of nodes into a object node
     * 
     * Merge a list of nodes into a object node, the object node
     * will be added directly under the lowest commom ancester of 
     * those nodes to be merged. 
     * 
     * @param ids a list of IDs of the node to be merged
     * @param onode the merged ObjectNode object 
     * @return the base-class pointer of the parent of merged nodes
     */
    NodeBase::Ptr mergeNodes(const std::vector<int> &, const ObjectNode &);

    /**
     * Merge a list of nodes into a concept node
     * 
     * Merge a list of nodes into a concept node, the concept node
     * will be added directly under the lowest commom ancester of 
     * those nodes to be merged. 
     * 
     * @param ptrs a list of pointers of the node to be merged
     * @param cnode the merged ConceptNode object 
     * @return the base-class pointer of the parent of merged nodes
     */
    NodeBase::Ptr mergeNodes(const std::vector<NodeBase::Ptr> &, const ConceptNode &);

    /**
     * Merge a list of nodes into a object node
     * 
     * Merge a list of nodes into a object node, the object node
     * will be added directly under the lowest commom ancester of 
     * those nodes to be merged. 
     * 
     * @param ptrs a list of pointers of the node to be merged
     * @param onode the merged ObjectNode object 
     * @return the base-class pointer of the parent of merged nodes
     */
    NodeBase::Ptr mergeNodes(const std::vector<NodeBase::Ptr> &, const ObjectNode &);
    

/*******************************************************************
 * Element Access
 ******************************************************************/
public:
    /**
     * Validate current parse graph
     * 
     * Check if node_dict edge_map are correct. This method could
     * be time-consuming.
     * 
     * @return true if valid, otherwise false
     */
    bool validate();


    /**
     * Get the pointer of the root node of the parse graph
     * 
     * @return the pointer of the root node
     */
    NodeBase::Ptr getRoot();


    /**
     * Get a node pointer according to its ID
     * 
     * @param id the ID the node
     * @return the pointer of the node, a `nullptr` is return
     *      if ID does not exist
     */
    NodeBase::Ptr getNode(int);

    /**
     * Find the direct parent of the given node
     * 
     * @param pnode the pointer of the node looks for parent
     * @return parent node pointer, `nullptr` if cannot find
     */
    NodeBase::Ptr findParentNode(NodeBase::Ptr);

    /**
     * Find the direct parent of the given node
     * 
     * @param id the ID of the node that looks for parent
     * @return parent node pointer, `nullptr` if cannot find
     */
    NodeBase::Ptr findParentNode(int);


    /**
     * Get all edges in the parse graph
     * 
     * Edge is define by a tuple (src_id, dst_id), edge from 
     * the source ID to the destination ID.
     * 
     * @return A vector of edge tuples 
     */
    std::vector<PgEdge> getEdges();


    /**
     * Get all node pointers in the parse graph
     * 
     * @return A vector of pointers of nodes in the parse graph
     */
    std::vector<NodeBase::Ptr> getNodes();


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
    std::string dump();


    /**
     * Save the parse graph to a file
     * 
     * @param filename the filename (string) of the output file
     * @return error code (int), 0 if succeed, otherwise error prompted
     */
    int save(const std::string);


/*******************************************************************
 * Internal Functions
 ******************************************************************/
private:
    /**
     * [Internal] Validate if current nodes match node_dict_
     * 
     * 
     * @return true if valid, otherwise false
     */
    bool validateNodes_();


    /**
     * [Internal] Validate if current edges match edge_map
     * 
     * 
     * @return true if valid, otherwise false
     */
    bool validateEdges_();


    /**
     * [Internal] Check whether a node exists or not
     * 
     * Note: this method is a more efficient way to check existance
     * 
     * @param id the id of node to be checked
     * @return true if exists, otherwise false
     */
    bool isNodeExist_(int);


    /**
     * [Internal] Check whether a node exists or not
     * 
     * @param pnode the pointer of node to be checked
     * @return true if exists, otherwise false
     */
    bool isNodeExist_(NodeBase::Ptr);


    /**
     * [Internal] Insert a node under parent node
     * 
     * @param parent the pointer of the parent node
     * @param child the pointer of the child node to be inserted
     * @return a base-class pointer of the inserted node
     */
    NodeBase::Ptr insertNode_(NodeBase::Ptr, NodeBase::Ptr);

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
    size_t edgeSetAdd_(const NodeBase::Ptr, const NodeBase::Ptr);


    /**
     * [Internal] Remove edges in the edge_map_
     * 
     * The edge between parent and child, and all descendants edges of child
     * will be removed
     * 
     * @param parent the parent node
     * @param child the child node
     * @return number of edges removed
     */
    size_t edgeSetRemove_(const NodeBase::Ptr, const NodeBase::Ptr);


    /**
     * [Internal] Add node to the node_dict_
     * 
     * Add node and all its descedants to the node_dict_.
     * 
     * @param pnode the parent node
     * @return number of nodes added
     */
    size_t nodeDictAdd_(NodeBase::Ptr);


    /**
     * [Internal] Remove a node and all its descendants in the node_dict_
     * 
     * The node and all its descendants will be removed
     * 
     * @param pnode the node to be removed
     * @return number of nodes removed
     */ 
    size_t nodeDictRemove_(NodeBase::Ptr);


    /**
     * [Internal] Set root node
     * 
     * Set the root node of the parse graph, and rebuild the node_dict
     * and the edge_map
     * 
     * @param root the pointer of the root node
     */
    void setRoot_(NodeBase::Ptr);


    /**
     * [Internal] Traverse all nodes started from pnode
     * 
     * @param pnode the root node where the traverse starts
     * @return a vector of NodeBase::Ptr that resides under pnode
     */ 
    std::vector<NodeBase::Ptr> traverseNode_(NodeBase::Ptr pnode);

    
    /**
     * [Internal] Traverse all edges started from pnode
     * 
     * @param pnode the root node where the traverse starts
     * @return a vector of tuple [(src, dst), ...] that contains all
     *     edges resides under the pnode
     */ 
    std::vector<std::vector<int> > traverseEdge_(NodeBase::Ptr pnode);

    
    /**
     * [Internal] Find the parent of the target node given the root
     * 
     * @param pnode the pointer of the target node that looks 
     *     for parent node
     * @param root the root the search tree
     * @return the parent node pointer of the target node, `nullptr`
     *     if cannot find
     */ 
    NodeBase::Ptr findParentNode_(NodeBase::Ptr, NodeBase::Ptr);


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
    NodeBase::Ptr mergeNodes_(const std::vector<NodeBase::Ptr> &, NodeBase::Ptr);

    
    /**
     * [Internal] Find the Lowest Common Ancester (LCA)
     * 
     * Find the lowest common ancester of the given nodes 
     * 
     * @param ptrs a list of pointers of the node to be merged
     * @return the pointer of the lowest common ancester
     */
    NodeBase::Ptr findLCA_(const std::vector<NodeBase::Ptr> &);


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
    NodeBase::Ptr lcaHelper_(NodeBase::Ptr, NodeBase::Ptr, NodeBase::Ptr);



    /**
     * [Internal] Generate JSON for edge information
     * 
     * @param edge PgEdge object of the edge
     * @return JSON object for ConceptNode/ObjectNode
     */
    nlohmann::json generateEdgeJson_(const PgEdge &);


    /**
     * [Internal] Generate JSON for node information
     * 
     * @param pnode the pointer of the target node 
     * @return JSON object for ConceptNode/ObjectNode
     */
    nlohmann::json generateNodeJson_(NodeBase::Ptr);

    /**
     * [Internal] Generate JSON for ConcepNode information
     * 
     * @param pnode the pointer of the target node 
     * @return JSON object
     *      {
     *          "node_type": "ConceptNode",
     *          "id": 123,
     *          "concept": "workspace"
     *      }
     */
    nlohmann::json generateConceptNodeJson_(ConceptNode::Ptr);


    /**
     * [Internal] Generate JSON for ObjectNode information
     * 
     * @param pnode the pointer of the target node 
     * @return JSON object
     *      {
     *          "node_type": "ObjectNode",
     *          "id": 123,
     *          "label": "desk"
     *      }
     */
    nlohmann::json generateObjectNodeJson_(ObjectNode::Ptr);


private:
    NodeBase::Ptr root_;
    PairMap<int, int, PgEdge> edge_map_;
    std::unordered_map<int, NodeBase::Ptr> node_dict_;
};

} // end of namespace pgm


#endif