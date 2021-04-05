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

class Chiplet_Mesh(SimpleTopology):
    description='Chiplet_Mesh'

    def __init__(self, controllers):
        self.nodes = controllers

    # Makes a generic mesh
    # assuming an equal number of cache and directory cntrls

    def makeTopology(self, options, network, IntLink, ExtLink, Router):
        nodes = self.nodes #TODO: How to specify only 64 nodes?

        num_routers = 80 #options.num_cpus
        #num_rows = options.mesh_rows

        # default values for link latency and router latency.
        # Can be over-ridden on a per link/router basis
        link_latency = options.link_latency # used by simple and garnet
        router_latency = options.router_latency # only used by garnet


        ## There must be an evenly divisible number of cntrls to routers
        ## Also, obviously the number or rows must be <= the number of routers
        cntrls_per_router, remainder = divmod(len(nodes), num_routers)
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
        for node_index in range(64):
            #if node_index < (len(nodes) - remainder):
            network_nodes.append(nodes[node_index])
            #else:
            #remainder_nodes.append(nodes[node_index])

        # Connect each node to the appropriate router
        ext_links = []
        for (i, n) in enumerate(network_nodes):
            cntrl_level, router_id = divmod(i, num_routers)
            assert(cntrl_level < cntrls_per_router)
            ext_links.append(ExtLink(link_id=link_count, ext_node=n,
                                    int_node=routers[router_id],
                                    latency = link_latency))
            link_count += 1

        ## Connect the remainding nodes to router 0.  These should only be
        ## DMA nodes.
        #for (i, node) in enumerate(remainder_nodes):
        #assert(node.type == 'DMA_Controller')
        #assert(i < remainder)
        #ext_links.append(ExtLink(link_id=link_count, ext_node=node,
        #int_node=routers[0],
        #latency = link_latency))

        #link_count += 1


        network.ext_links = ext_links

        # Create the mesh links.
        int_links = []

        # East output to West input links (weight = 1)
        east_out = []
        for i in range(0, 63, 2): #All even cores send from east
            east_out.append(i)
        west_in = []
        for i in range(1, 64, 2): #Al odd cores receive from west
            west_in.append(i)
        for i in range(len(east_out)):
            int_links.append(IntLink(link_id=link_count,
                                     src_node=routers[east_out[i]],
                                     dst_node=routers[west_in[i]],
                                     src_outport="East",
                                     dst_inport="West",
                                     latency = link_latency,
                                     weight=1))
            link_count += 1

        # West output to East input links (weight = 1)
        east_in = []
        for i in range(0, 63, 2): #All even cores receive from east
            east_in.append(i)
        west_out = []
        for i in range(1, 64, 2): #All odd cores send from west
            west_out.append(i)
        for i in range(len(west_out)):
            int_links.append(IntLink(link_id=link_count,
                                     src_node=routers[west_out[i]],
                                     dst_node=routers[east_in[i]],
                                     src_outport="West",
                                     dst_inport="East",
                                     latency = link_latency,
                                     weight=1))
            link_count += 1

        # North output to South input links (weight = 2)
        north_out = []
        south_in = []
        for i in range(0, 63, 4):
            north_out.append(i) #All bottom cores in chiplet send from north
            north_out.append(i+1)
            south_in.append(i+2) #All top cores in chiplet receive from south
            south_in.append(i+3)
        for i in range(len(north_out)):
            int_links.append(IntLink(link_id=link_count,
                                     src_node=routers[north_out[i]],
                                     dst_node=routers[south_in[i]],
                                     src_outport="North",
                                     dst_inport="South",
                                     latency = link_latency,
                                     weight=2))
            link_count += 1

        # South output to North input links (weight = 2)
        south_out = []
        north_in = []
        for i in range(0, 63, 4):
            north_in.append(i) #All bottom cores in chiplet receive from north
            north_in.append(i+1)
            south_out.append(i+2) #All top cores in chiplet send from south
            south_out.append(i+3)
        for i in range(len(south_out)):
            int_links.append(IntLink(link_id=link_count,
                                     src_node=routers[south_out[i]],
                                     dst_node=routers[north_in[i]],
                                     src_outport="South",
                                     dst_inport="North",
                                     latency = link_latency,
                                     weight=2))
            link_count += 1

        # CMESH LINKS
        # East output to West input links (weight = 3)
        east_out_cmesh = []
        west_in_cmesh = []
        for i in range(64, 79, 4):
            east_out_cmesh.append(i)
            east_out_cmesh.append(i+1)
            east_out_cmesh.append(i+2)
            west_in_cmesh.append(i+1)
            west_in_cmesh.append(i+2)
            west_in_cmesh.append(i+3)
        for i in range(len(east_out_cmesh)):
            int_links.append(IntLink(link_id=link_count,
                                     src_node=routers[east_out_cmesh[i]],
                                     dst_node=routers[west_in_cmesh[i]],
                                     src_outport="East",
                                     dst_inport="West",
                                     latency = link_latency,
                                     weight=3))
            link_count += 1

        # West output to East input links (weight = 3)
        west_out_cmesh = []
        east_in_cmesh = []
        for i in range(64, 79, 4):
            east_in_cmesh.append(i)
            east_in_cmesh.append(i+1)
            east_in_cmesh.append(i+2)
            west_out_cmesh.append(i+1)
            west_out_cmesh.append(i+2)
            west_out_cmesh.append(i+3)
        for i in range(len(west_out_cmesh)):
            int_links.append(IntLink(link_id=link_count,
                                     src_node=routers[west_out_cmesh[i]],
                                     dst_node=routers[east_in_cmesh[i]],
                                     src_outport="West",
                                     dst_inport="East",
                                     latency = link_latency,
                                     weight=3))
            link_count += 1

        # North output to South input links (weight = 3)
        north_out_cmesh = []
        south_in_cmesh = []
        for i in range(64, 76):
            north_out_cmesh.append(i)
        for i in range(68, 80):
            south_in_cmesh.append(i)
        for i in range(len(north_out_cmesh)):
            int_links.append(IntLink(link_id=link_count,
                                     src_node=routers[north_out_cmesh[i]],
                                     dst_node=routers[south_in_cmesh[i]],
                                     src_outport="North",
                                     dst_inport="South",
                                     latency = link_latency,
                                     weight=3))
            link_count += 1

        # South output to North input links (weight = 3)
        north_in_cmesh = []
        south_out_cmesh = []
        for i in range(64, 76):
            north_in_cmesh.append(i)
        for i in range(68, 80):
            south_out_cmesh.append(i)
        for i in range(len(south_out_cmesh)):
            int_links.append(IntLink(link_id=link_count,
                                     src_node=routers[south_out_cmesh[i]],
                                     dst_node=routers[north_in_cmesh[i]],
                                     src_outport="South",
                                     dst_inport="North",
                                     latency = link_latency,
                                     weight=3))
            link_count += 1

        network.int_links = int_links

    # Register nodes with filesystem
    def registerTopology(self, options):
        for i in range(options.num_cpus):
            FileSystemConfig.register_node([i],
                    MemorySize(options.mem_size) // options.num_cpus, i)
