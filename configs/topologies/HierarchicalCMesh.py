# Copyright (c) 2010 Advanced Micro Devices, Inc.
#               2016 Georgia Institute of Technology
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from m5.params import *
from m5.objects import *

from common import FileSystemConfig

from topologies.BaseTopology import SimpleTopology

# Creates a generic Mesh assuming an equal number of cache
# and directory controllers.
# XY routing is enforced (using link weights)
# to guarantee deadlock freedom.

class HierarchicalCMesh(SimpleTopology):
    description='HierarchicalCMesh'

    def __init__(self, controllers):
        self.nodes = controllers

    # Makes a generic mesh
    # assuming an equal number of cache and directory cntrls

    def makeTopology(self, options, network, IntLink, ExtLink, Router):
        nodes = self.nodes #TODO: How to specify only 64 nodes?

        chiplet_dim = 2
        cpus_per_chiplet = chiplet_dim ** 2

        num_cpus = options.num_cpus
        num_cpu_routers = num_cpus # may need to change later based on cpus/rtr
        num_mesh_routers = num_cpu_routers
        assert(num_cpu_routers == 64) # this is a hardcoded check for now
        num_non_cpu_routers = int(num_cpu_routers / cpus_per_chiplet)
        num_cmesh_routers = num_non_cpu_routers
        assert(num_non_cpu_routers == 16) # this is a hardcoded check for now
        num_routers = num_cpu_routers + num_non_cpu_routers
        assert(num_routers == 80) # this is a hardcoded check for now
        num_rows = options.mesh_rows
        cpu_rtrs_per_col = num_rows
        mesh_rtrs_per_col = num_rows
        assert(num_rows == 8) # this is a hardcoded check for now
        num_cols = int(num_cpus / num_rows)
        cpu_rtrs_per_row = num_cols
        mesh_rtrs_per_row = num_cols
        assert(mesh_rtrs_per_row == 8)

        cmesh_rows = int(num_rows / chiplet_dim)
        assert(cmesh_rows == 4) # this is a hardcoded check for now
        cmesh_cols = int(num_cols / chiplet_dim)
        assert(cmesh_cols == 4) # this is a hardcoded check for now
        cmesh_rtrs_per_row = cmesh_cols
        cmesh_rtrs_per_col = cmesh_rows

        # must be square
        assert(num_rows ** 2 == options.num_cpus)

        # chiplet mesh constants
        mesh_rtr_base_id = 0
        mesh_rtr_max_id = num_cpu_routers - 1
        assert(mesh_rtr_max_id == 63) # this is a hardcoded check for now
        mesh_rtr_max_id_excl = mesh_rtr_max_id + 1
        assert(mesh_rtr_max_id_excl == 64) # this is a hardcoded check for now

        # cmesh constants
        cmesh_rtr_base_id = mesh_rtr_max_id + 1
        assert(cmesh_rtr_base_id == 64) # this is a hardcoded check for now
        cmesh_rtr_max_id = cmesh_rtr_base_id + num_cmesh_routers - 1
        assert(cmesh_rtr_max_id == 79) # this is a hardcoded check for now
        cmesh_rtr_max_id_excl = cmesh_rtr_max_id + 1
        assert(cmesh_rtr_max_id_excl == 80) # this is a hardcoded check for now

        # default values for link latency and router latency.
        # Can be over-ridden on a per link/router basis
        link_latency = options.link_latency # used by simple and garnet
        router_latency = options.router_latency # only used by garnet


        ## There must be an evenly divisible number of cntrls to routers
        ## Also, obviously the number or rows must be <= the number of routers
        cntrls_per_router, remainder = divmod(len(nodes), num_cpu_routers)
        print(cntrls_per_router)
        #assert(num_rows > 0 and num_rows <= num_routers)
        #num_columns = int(num_routers / num_rows)
        #assert(num_columns * num_rows == num_routers)

        # Create the routers in the mesh
        routers = [Router(router_id=i, latency = router_latency) \
            for i in range(num_routers)]
        network.routers = routers

        # link counter to set unique link ids
        link_count = 0

        # Add all but the remainder nodes to the list of nodes to be uniformly
        # distributed across the network.
        network_nodes = []
        #remainder_nodes = []
        for node_index in range(128):
            #if node_index < (len(nodes) - remainder):
            network_nodes.append(nodes[node_index])
            #else:
                #remainder_nodes.append(nodes[node_index])

        # Connect each node to the appropriate router
        # this is currently one L1 and one directory controller per router
        ext_links = []
        for (i, n) in enumerate(network_nodes):
            cntrl_level, router_id = divmod(i, num_cpu_routers)
            assert(cntrl_level < cntrls_per_router)
            ext_links.append(ExtLink(link_id=link_count, ext_node=n,
                                    int_node=routers[router_id],
                                    latency = link_latency))
            link_count += 1

        ## Connect the remainding nodes to router 0.  These should only be
        ## DMA nodes.
        #for (i, node) in enumerate(remainder_nodes):
        #    assert(node.type == 'DMA_Controller')
        #    assert(i < remainder)
        #    ext_links.append(ExtLink(link_id=link_count, ext_node=node,
        #        int_node=routers[0],
        #        latency = link_latency))

        #    link_count += 1


        network.ext_links = ext_links


        debug_con = [] # format ((src, dest), link_id)
        def debug_con_app_chk(src, dest, lid, con):
            entry = ((src, dest), con)
            for dc in con:
                if dc[0] == entry[0]:
                    assert(f"{entry} is a dupliucate. OL: {dc}")

            con.append(entry)

        ########################
        #  ON-CHIPLET XY MESH  #
        ########################

        on_chiplet_link_latency = link_latency
        on_chiplet_link_width = 32
        chiplet_width = 2

        # Create the mesh links.
        int_links = []

        # on-chiplet EW and WE links
        for i in range(mesh_rtr_base_id, num_mesh_routers, chiplet_width):
            # add east to west link (left to right)
            debug_con_app_chk(i, i + 1, link_count, debug_con)
            int_links.append(IntLink(link_id=link_count,
                                     src_node=routers[i],
                                     dst_node=routers[i + 1],
                                     src_outport="East",
                                     dst_inport="West",
                                     latency=link_latency))

            link_count += 1

            # add west to east link (right to left)
            debug_con_app_chk(i + 1, i, link_count, debug_con)
            int_links.append(IntLink(link_id=link_count,
                                     src_node=routers[i + 1],
                                     dst_node=routers[i],
                                     src_outport="West",
                                     dst_inport="East",
                                     latency=link_latency))

            link_count += 1

        for base_id in range(mesh_rtr_base_id, num_mesh_routers, \
                mesh_rtrs_per_row * chiplet_dim):
            for i in range(base_id, base_id + mesh_rtrs_per_row, 1):
                south_id = i
                north_id = i + mesh_rtrs_per_row

                # add south to north link (bottom to top)
                debug_con_app_chk(south_id, north_id, link_count, debug_con)
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[south_id],
                                         dst_node=routers[north_id],
                                         src_outport="North",
                                         dst_inport="South",
                                         latency = link_latency))
                link_count += 1

                # add north to south link (top to bottom)
                debug_con_app_chk(north_id, south_id, link_count, debug_con)
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[north_id],
                                         dst_node=routers[south_id],
                                         src_outport="South",
                                         dst_inport="North",
                                         latency = link_latency))
                link_count += 1


        #################
        #  CMESH LINKS  #
        #################

        # CMESH LINKS
        for cmesh_row_base_id in range(cmesh_rtr_base_id, \
                cmesh_rtr_max_id_excl, cmesh_rtrs_per_row):
            for i in range(cmesh_row_base_id, \
                    cmesh_row_base_id + cmesh_rtrs_per_row - 1):
                # add east to west link (left to right)
                debug_con_app_chk(i, i + 1, link_count, debug_con)
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[i],
                                         dst_node=routers[i + 1],
                                         src_outport="East",
                                         dst_inport="West",
                                         latency=link_latency))

                link_count += 1

                # add west to east link (right to left)
                debug_con_app_chk(i + 1, i, link_count, debug_con)
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[i + 1],
                                         dst_node=routers[i],
                                         src_outport="West",
                                         dst_inport="East",
                                         latency=link_latency))

                link_count += 1

        for cmesh_row_base_id in range(cmesh_rtr_base_id, \
                cmesh_rtr_max_id_excl - cmesh_rtrs_per_row, \
                cmesh_rtrs_per_row):
            for i in range(cmesh_row_base_id, cmesh_row_base_id \
                    + cmesh_rtrs_per_row):
                south_id = i
                north_id = i + cmesh_rtrs_per_row

                # add south to north link (bottom to top)
                debug_con_app_chk(south_id, north_id, link_count, debug_con)
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[south_id],
                                         dst_node=routers[north_id],
                                         src_outport="North",
                                         dst_inport="South",
                                         latency = link_latency))
                link_count += 1

                # add north to south link (top to bottom)
                debug_con_app_chk(north_id, south_id, link_count, debug_con)
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[north_id],
                                         dst_node=routers[south_id],
                                         src_outport="South",
                                         dst_inport="North",
                                         latency = link_latency))
                link_count += 1


        #############################
        #  CMESH <-> MESH INTERCON  #
        #############################

        def mesh_xy_to_mesh_id(x, y):
            return mesh_rtr_base_id + ((y * mesh_rtrs_per_row) + x)

        def mesh_xy_to_cmesh_xy(x, y):
            cmesh_x = int(x / chiplet_dim)
            cmesh_y = int(y / chiplet_dim)
            return (cmesh_x, cmesh_y)

        def cmesh_xy_to_cmesh_id(x, y):
            return cmesh_rtr_base_id + ((y * cmesh_rtrs_per_row) + x)

        def mesh_xy_to_quadrant(x, y):
            if x % 2 == 0 and y % 2 == 0:
                return "SW"
            elif x % 2 == 1 and y % 2 == 0:
                return "SE"
            elif x % 2 == 0 and y % 2 == 1:
                return "NW"
            elif x % 2 == 1 and y % 2 == 1:
                return "NE"
            else:
                assert(False) # impossible

        for x in range(0, mesh_rtrs_per_row):
            for y in range(0, mesh_rtrs_per_col):
                mesh_id = mesh_xy_to_mesh_id(x, y)
                mesh_quad = mesh_xy_to_quadrant(x, y)

                cx, cy = mesh_xy_to_cmesh_xy(x, y)
                cmesh_id = cmesh_xy_to_cmesh_id(cx, cy)

                quad_port_name = "Mesh_" + mesh_quad

                # add mesh to cmesh link (promotion link)
                debug_con_app_chk(mesh_id, cmesh_id, link_count, debug_con)
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[mesh_id],
                                         dst_node=routers[cmesh_id],
                                         src_outport="Cmesh",
                                         dst_inport=quad_port_name + "i",
                                         latency = link_latency))
                link_count += 1

                # add cmesh to mesh link (demotion link)
                debug_con_app_chk(cmesh_id, mesh_id, link_count, debug_con)
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[cmesh_id],
                                         dst_node=routers[mesh_id],
                                         src_outport=quad_port_name,
                                         dst_inport="Cmesh",
                                         latency = link_latency))
                link_count += 1

        #assert False, f"num_links: {len(debug_con)} --- {debug_con}"

        network.int_links = int_links

    # Register nodes with filesystem
    def registerTopology(self, options):
        for i in range(options.num_cpus):
            FileSystemConfig.register_node([i],
                    MemorySize(options.mem_size) // options.num_cpus, i)
