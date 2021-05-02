/*
 * Copyright (c) 2008 Princeton University
 * Copyright (c) 2016 Georgia Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "mem/ruby/network/garnet/RoutingUnit.hh"

#include "base/cast.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet/InputUnit.hh"
#include "mem/ruby/network/garnet/Router.hh"
#include "mem/ruby/slicc_interface/Message.hh"

RoutingUnit::RoutingUnit(Router *router)
{
    m_router = router;
    m_routing_table.clear();
    m_weight_table.clear();
}

void
RoutingUnit::addRoute(std::vector<NetDest>& routing_table_entry)
{
    if (routing_table_entry.size() > m_routing_table.size()) {
        m_routing_table.resize(routing_table_entry.size());
    }
    for (int v = 0; v < routing_table_entry.size(); v++) {
        m_routing_table[v].push_back(routing_table_entry[v]);
    }
}

void
RoutingUnit::addWeight(int link_weight)
{
    m_weight_table.push_back(link_weight);
}

bool
RoutingUnit::supportsVnet(int vnet, std::vector<int> sVnets)
{
    // If all vnets are supported, return true
    if (sVnets.size() == 0) {
        return true;
    }

    // Find the vnet in the vector, return true
    if (std::find(sVnets.begin(), sVnets.end(), vnet) != sVnets.end()) {
        return true;
    }

    // Not supported vnet
    return false;
}

/*
 * This is the default routing algorithm in garnet.
 * The routing table is populated during topology creation.
 * Routes can be biased via weight assignments in the topology file.
 * Correct weight assignments are critical to provide deadlock avoidance.
 */
int
RoutingUnit::lookupRoutingTable(int vnet, NetDest msg_destination)
{
    // First find all possible output link candidates
    // For ordered vnet, just choose the first
    // (to make sure different packets don't choose different routes)
    // For unordered vnet, randomly choose any of the links
    // To have a strict ordering between links, they should be given
    // different weights in the topology file

    int output_link = -1;
    int min_weight = INFINITE_;
    std::vector<int> output_link_candidates;
    int num_candidates = 0;

    // Identify the minimum weight among the candidate output links
    for (int link = 0; link < m_routing_table[vnet].size(); link++) {
        if (msg_destination.intersectionIsNotEmpty(
            m_routing_table[vnet][link])) {

        if (m_weight_table[link] <= min_weight)
            min_weight = m_weight_table[link];
        }
    }

    // Collect all candidate output links with this minimum weight
    for (int link = 0; link < m_routing_table[vnet].size(); link++) {
        if (msg_destination.intersectionIsNotEmpty(
            m_routing_table[vnet][link])) {

            if (m_weight_table[link] == min_weight) {
                num_candidates++;
                output_link_candidates.push_back(link);
            }
        }
    }

    if (output_link_candidates.size() == 0) {
        fatal("Fatal Error:: No Route exists from this Router.");
        exit(0);
    }

    // Randomly select any candidate output link
    int candidate = 0;
    if (!(m_router->get_net_ptr())->isVNetOrdered(vnet))
        candidate = rand() % num_candidates;

    output_link = output_link_candidates.at(candidate);
    return output_link;
}


void
RoutingUnit::addInDirection(PortDirection inport_dirn, int inport_idx)
{
    m_inports_dirn2idx[inport_dirn] = inport_idx;
    m_inports_idx2dirn[inport_idx]  = inport_dirn;
}

void
RoutingUnit::addOutDirection(PortDirection outport_dirn, int outport_idx)
{
    m_outports_dirn2idx[outport_dirn] = outport_idx;
    m_outports_idx2dirn[outport_idx]  = outport_dirn;
}

// outportCompute() is called by the InputUnit
// It calls the routing table by default.
// A template for adaptive topology-specific routing algorithm
// implementations using port directions rather than a static routing
// table is provided here.

int
RoutingUnit::outportCompute(RouteInfo route, int inport,
                            PortDirection inport_dirn)
{
    int outport = -1;

    if (route.dest_router == m_router->get_id()) {

        // Multiple NIs may be connected to this router,
        // all with output port direction = "Local"
        // Get exact outport id from table
        outport = lookupRoutingTable(route.vnet, route.net_dest);
        return outport;
    }

    // Routing Algorithm set in GarnetNetwork.py
    // Can be over-ridden from command line using --routing-algorithm = 1
    RoutingAlgorithm routing_algorithm =
        (RoutingAlgorithm) m_router->get_net_ptr()->getRoutingAlgorithm();

    switch (routing_algorithm) {
        case TABLE_:  outport =
            lookupRoutingTable(route.vnet, route.net_dest); break;
        case XY_:     outport =
            outportComputeXY(route, inport, inport_dirn); break;
        // any custom algorithm
        case CUSTOM_: outport =
            outportComputeCustom(route, inport, inport_dirn); break;
        default: outport =
            lookupRoutingTable(route.vnet, route.net_dest); break;
    }

    assert(outport != -1);
    return outport;
}

// XY routing implemented using port directions
// Only for reference purpose in a Mesh
// By default Garnet uses the routing table
int
RoutingUnit::outportComputeXY(RouteInfo route,
                              int inport,
                              PortDirection inport_dirn)
{
    PortDirection outport_dirn = "Unknown";

    M5_VAR_USED int num_rows = m_router->get_net_ptr()->getNumRows();
    int num_cols = m_router->get_net_ptr()->getNumCols();
    assert(num_rows > 0 && num_cols > 0);

    num_rows = 8;
    num_cols = 8;

    int my_id = m_router->get_id();
    int my_x = my_id % num_cols;
    int my_y = my_id / num_cols;

    int dest_id = route.dest_router;
    int dest_x = dest_id % num_cols;
    int dest_y = dest_id / num_cols;

    int x_hops = abs(dest_x - my_x);
    int y_hops = abs(dest_y - my_y);

    bool x_dirn = (dest_x >= my_x);
    bool y_dirn = (dest_y >= my_y);

    // already checked that in outportCompute() function
    assert(!(x_hops == 0 && y_hops == 0));

    if (x_hops > 0) {
        if (x_dirn) {
            fatal_if(!(inport_dirn == "CMesh"
                                    || inport_dirn == "Local"
                                    || inport_dirn == "West"),
                            inport_dirn
                            + ", cur ID: "
                            + std::to_string(my_id)
                            + ", src ID: "
                            + std::to_string(route.src_router)
                            + ", dest ID: "
                            + std::to_string(route.dest_router));
            outport_dirn = "East";
        } else {
            fatal_if(!(inport_dirn == "CMesh"
                                    || inport_dirn == "Local"
                                    || inport_dirn == "East"),
                            inport_dirn
                            + ", cur ID: "
                            + std::to_string(my_id)
                            + ", src ID: "
                            + std::to_string(route.src_router)
                            + ", dest ID: "
                            + std::to_string(route.dest_router));
            outport_dirn = "West";
        }
    } else if (y_hops > 0) {
        if (y_dirn) {
            // "Local" or "South" or "West" or "East"
            assert(inport_dirn != "North");
            outport_dirn = "North";
        } else {
            // "Local" or "North" or "West" or "East"
            assert(inport_dirn != "South");
            outport_dirn = "South";
        }
    } else {
        // x_hops == 0 and y_hops == 0
        // this is not possible
        // already checked that in outportCompute() function
        panic("x_hops == y_hops == 0");
    }

        //inform("localXY output direction: " + outport_dirn);

    return m_outports_dirn2idx[outport_dirn];
}

// Template for implementing custom routing algorithm
// using port directions. (Example adaptive)
int
RoutingUnit::outportComputeCustom(RouteInfo route,
                                 int inport,
                                 PortDirection inport_dirn)
{
    PortDirection outport_dirn = "Unknown";

    inform("entering outportComputeCustom");

    M5_VAR_USED int num_rows = m_router->get_net_ptr()->getNumRows();
    int num_cols = m_router->get_net_ptr()->getNumCols();
    assert(num_rows > 0 && num_cols > 0);

    // override topol doesn't support intermediate routers
    num_rows = 8;
    num_cols = 8;

    // ahh garnet params are weird and num_cols is being inferred
    // from rtr count not cpus / cpus/rtr count so its wrong
    //int base_cmesh_rtr_id = num_rows * num_cols;
    int base_cmesh_rtr_id = 64;
    const int chiplet_cols = 2;
    const int chiplet_rows = 2;
    const int cmesh_cols = 4;
    const int cmesh_rows = 4;

    // destination must be in the mesh (e.g. cmesh dests are not
    // permitted
    int dest_id = route.dest_router;
    assert(dest_id <= base_cmesh_rtr_id);

    int dest_x = dest_id % num_cols;
    int dest_y = dest_id / num_cols;
    int dest_chiplet_x = dest_x / chiplet_rows;
    int dest_chiplet_y = dest_y / chiplet_cols;


    int my_id = m_router->get_id();
    if (my_id >= base_cmesh_rtr_id) {
        int my_x_cmesh = (my_id - base_cmesh_rtr_id) % cmesh_cols;
        int my_y_cmesh = (my_id - base_cmesh_rtr_id) / cmesh_cols;

        if (my_x_cmesh == dest_chiplet_x
                    && my_y_cmesh == dest_chiplet_y) {
                // were on the cmesh but want to go to the mesh at the current
                // chiplet compute dest quadrant
                if (dest_x % 2 == 0 && dest_y % 2 == 0)
                        outport_dirn = "Mesh_SW";
                        // lower left corner is even row and col
                else if (dest_x % 2 == 1 && dest_y % 2 == 0)
                        outport_dirn = "Mesh_SE";
                        // upper left corner is odd row and even col
                else if (dest_x % 2 == 0 && dest_y % 2 == 1)
                        outport_dirn = "Mesh_NW";
                        // lower right corner is even row and odd col
                else if (dest_x % 2 == 1 && dest_y % 2 == 1)
                        outport_dirn = "Mesh_NE";
                        // upper right corner is even row and even col
            } else {

                int cmesh_x_hops = abs(dest_chiplet_x - my_x_cmesh);
                int cmesh_y_hops = abs(dest_chiplet_y - my_y_cmesh);

                bool cmesh_x_dirn = (dest_chiplet_x >= my_x_cmesh);
                bool cmesh_y_dirn = (dest_chiplet_y >= my_y_cmesh);

                // already checked that in outportCompute() function
                //assert(!(x_hops == 0 && y_hops == 0));

                if (cmesh_x_hops > 0) {
                    if (cmesh_x_dirn) {
                        //assert(inport_dirn == "Local"
                        //|| inport_dirn == "West");
                        outport_dirn = "East";
                    } else {
                        //assert(inport_dirn == "Local"
                        //|| inport_dirn == "East");
                        outport_dirn = "West";
                    }
                } else if (cmesh_y_hops > 0) {
                    if (cmesh_y_dirn) {
                        // "Local" or "South" or "West" or "East"
                        //assert(inport_dirn != "North");
                        outport_dirn = "North";
                    } else {
                        // "Local" or "North" or "West" or "East"
                        //assert(inport_dirn != "South");
                        outport_dirn = "South";
                    }
                } else {
                    // x_hops == 0 and y_hops == 0
                    // this is not possible
                    // already checked that in outportCompute() function
                    panic("x_hops == y_hops == 0");
                }
        }
    } else {
        // currently in local chiplet
        int my_x = my_id % num_cols;
        int my_y = my_id / num_cols;
        int my_x_cmesh = my_x / chiplet_cols;
        int my_y_cmesh = my_y / chiplet_rows;

        if (my_x_cmesh == dest_chiplet_x
                        && my_y_cmesh == dest_chiplet_y) {
                // on chiplet and dest chiplet is the same
                fatal_if(my_id >= 64, std::to_string(base_cmesh_rtr_id));
                // were on the same chiplet and on the chiplet mesh
                // we can invoke the local XY router
                auto ret = outportComputeXY(route, inport, inport_dirn);
                inform("leaving outport compute custom localXY: "
                                + std::to_string(ret) + ", cur ID: "
                                + std::to_string(my_id) + ", src: "
                                + std::to_string(route.src_router) + ", dest: "
                                + std::to_string(dest_id));
                return ret;
                // NOTE inport not used, inport_dirn used for assert checks
        } else {
                // we must be on a chiplet mesh, but src and dest are different
                // chiplets, we need to promote the pakcet to the cmesh
                // TODO compute quadrant??? no i don't think so
                outport_dirn = "Cmesh";
        }
    }

        // if on chiplet
        // 	call outputComputeXY
        // else if on chiplet mesh
        // 	set outport cmesh
        // 	if identical inbound port names not allowed
        // 		claulate current chiplet quadrant
        // 		set outport cmesh inbound quadrant
        // else if on chmesh and dest not current cmesh chiplet rtr
        // 	invoke cmesh xy
        // else (we must be on teh cmesh at the correct chiplet)
        // 	calculate chiplet quadrant
        // 	set appropriate inbound link

    auto ret = m_outports_dirn2idx[outport_dirn];
    inform("leaving outport compute custom glb: " + std::to_string(ret)
                    + ", cur ID: " + std::to_string(my_id)
                    + ", dir: " + outport_dirn+ std::to_string(dest_id));
    return ret;
}
