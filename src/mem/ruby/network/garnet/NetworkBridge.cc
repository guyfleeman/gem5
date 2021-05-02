/*
 * Copyright (c) 2020 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "mem/ruby/network/garnet/NetworkBridge.hh"

#include <cmath>

#include "debug/RubyNetwork.hh"
#include "params/GarnetIntLink.hh"

NetworkBridge::NetworkBridge(const Params &p)
    :CreditLink(p)
{
    enCdc = true;
    enSerDes = true;
    mType = p.vtype;

    cdcLatency = p.cdc_latency;
    serDesLatency = p.serdes_latency;
    lastScheduledAt = 0;

    nLink = p.link;
    if (mType == Enums::LINK_OBJECT) {
        nLink->setLinkConsumer(this);
        setSourceQueue(nLink->getBuffer(), nLink);
    } else if (mType == Enums::OBJECT_LINK) {
        nLink->setSourceQueue(&linkBuffer, this);
        setLinkConsumer(nLink);
    } else {
        // CDC type must be set
        panic("CDC type must be set");
    }
}

void
NetworkBridge::setVcsPerVnet(uint32_t consumerVcs)
{
    DPRINTF(RubyNetwork, "VcsPerVnet VC: %d\n", consumerVcs);
    NetworkLink::setVcsPerVnet(consumerVcs);
    lenBuffer.resize(consumerVcs * m_virt_nets);
    sizeSent.resize(consumerVcs * m_virt_nets);
    flitsSent.resize(consumerVcs * m_virt_nets);
    extraCredit.resize(consumerVcs * m_virt_nets);

    nLink->setVcsPerVnet(consumerVcs);
}

void
NetworkBridge::initBridge(NetworkBridge *coBrid, bool cdc_en, bool serdes_en)
{
    coBridge = coBrid;
    enCdc = cdc_en;
    enSerDes = serdes_en;
}

NetworkBridge::~NetworkBridge()
{
}

void
NetworkBridge::scheduleFlit(flit *t_flit, Cycles latency)
{
    Cycles totLatency = latency;

    if (enCdc) {
        // Add the CDC latency
        totLatency = latency + cdcLatency;
    }

    Tick sendTime = link_consumer->getObject()->clockEdge(totLatency);
    Tick nextAvailTick = lastScheduledAt + link_consumer->getObject()->\
            cyclesToTicks(Cycles(1));
    sendTime = std::max(nextAvailTick, sendTime);
    t_flit->set_time(sendTime);
    lastScheduledAt = sendTime;
    linkBuffer.insert(t_flit);
    link_consumer->scheduleEventAbsolute(sendTime);
}

void
NetworkBridge::neutralize(int vc, int eCredit)
{
    extraCredit[vc].push(eCredit);
}

void
NetworkBridge::flitisizeAndSend(flit *t_flit)
{
    // Serialize-Deserialize only if it is enabled
    if (enSerDes) {
        // Calculate the target-width
        int target_width = bitWidth;
        int cur_width = nLink->bitWidth;
        if (mType == Enums::OBJECT_LINK) {
            target_width = nLink->bitWidth;
            cur_width = bitWidth;
        }

        DPRINTF(RubyNetwork, "Target width: %d Current: %d\n",
            target_width, cur_width);
        assert(target_width != cur_width);

        int vc = t_flit->get_vc();

        if (target_width > cur_width) {
            // Deserialize
            // This deserializer combines flits from the
            // same message together
            int num_flits = 0;
            int flitPossible = 0;
            if (t_flit->get_type() == CREDIT_) {
                lenBuffer[vc]++;
                assert(extraCredit[vc].front());
                printf("***** Deserializer Rx Credit, EC front val: %d\r\n",
                                extraCredit[vc].front());
                if (lenBuffer[vc] == extraCredit[vc].front()) {
                    printf("***** Deser EC==LB, EC front val: %d\r\n",
                                    lenBuffer[vc]);
                    flitPossible = 1;
                    extraCredit[vc].pop();
                    lenBuffer[vc] = 0;
                }
            } else if (t_flit->get_type() == TAIL_ ||
                       t_flit->get_type() == HEAD_TAIL_) {
                // If its the end of packet, then send whatever
                // is available.
                int sizeAvail = (t_flit->msgSize - sizeSent[vc]);
                flitPossible = ceil((float)sizeAvail/(float)target_width);
                assert (flitPossible < 2);
                num_flits = (t_flit->get_id() + 1) - flitsSent[vc];
                // Stop tracking the packet.
                flitsSent[vc] = 0;
                sizeSent[vc] = 0;
            } else {
                // If we are yet to receive the complete packet
                // track the size recieved and flits deserialized.
                int sizeAvail =
                    ((t_flit->get_id() + 1)*cur_width) - sizeSent[vc];
                flitPossible = floor((float)sizeAvail/(float)target_width);
                assert (flitPossible < 2);
                num_flits = (t_flit->get_id() + 1) - flitsSent[vc];
                if (flitPossible) {
                    sizeSent[vc] += target_width;
                    flitsSent[vc] = t_flit->get_id() + 1;
                }
            }

            DPRINTF(RubyNetwork, "Deserialize :%dB -----> %dB "
                " vc:%d\n", cur_width, target_width, vc);

            flit *fl = NULL;
            if (flitPossible) {
                fl = t_flit->deserialize(lenBuffer[vc], num_flits,
                    target_width);
            }

            // Inform the credit serializer about the number
            // of flits that were generated.
            if (t_flit->get_type() != CREDIT_ && fl) {
                printf("***** Deserializer CoBridge Neutralize, val: %d\r\n",
                                num_flits);
                coBridge->neutralize(vc, num_flits);
            }

            // Schedule only if we are done deserializing
            if (fl) {
                DPRINTF(RubyNetwork, "Scheduling a flit\n");
                lenBuffer[vc] = 0;
                scheduleFlit(fl, serDesLatency);
            }
            // Delete this flit, new flit is sent in any case
            delete t_flit;
        } else {
            // Serialize
            DPRINTF(RubyNetwork, "Serializing flit :%d -----> %d "
            "(vc:%d, Original Message Size: %d)\n",
                cur_width, target_width, vc, t_flit->msgSize);

            int flitPossible = 0;
            if (t_flit->get_type() == CREDIT_) {
                // We store the deserialization ratio and then
                // access it when serializing credits in the
                // oppposite direction.
                assert(extraCredit[vc].front());
                flitPossible = extraCredit[vc].front();
                extraCredit[vc].pop();
                printf("***** Serializer Rx Credit, EC front val: %d\r\n",
                                flitPossible);
            } else if (t_flit->get_type() == HEAD_ ||
                    t_flit->get_type() == BODY_) {
                // id is the index into the packet
                // its weird that sizeAvail seems to go up as id goes up, but
                // this is compensated for in (- sizeSent[vc]) which is zeroed
                // out when the tail comes along
                // cur_wdith = fatter link
                // target_width = narrower link
                // flit
                int sizeAvail =
                    ((t_flit->get_id() + 1)*cur_width) - sizeSent[vc];
                flitPossible = floor((float)sizeAvail/(float)target_width);
                if (flitPossible) {
                    sizeSent[vc] += flitPossible*target_width;
                    flitsSent[vc] += flitPossible;
                }
            } else {
                // type = TAIL_
                int sizeAvail = t_flit->msgSize - sizeSent[vc];
                flitPossible = ceil((float)sizeAvail/(float)target_width);
                sizeSent[vc] = 0;
                flitsSent[vc] = 0;
            }
            assert(flitPossible > 0);

            // Schedule all the flits
            // num_flits could be zero for credits
            for (int i = 0; i < flitPossible; i++) {
                // Ignore neutralized credits
                flit *fl = t_flit->serialize(i, flitPossible, target_width);
                scheduleFlit(fl, serDesLatency);
                DPRINTF(RubyNetwork, "Serialized to flit[%d of %d parts]:"
                " %s\n", i+1, flitPossible, *fl);
            }

            if (t_flit->get_type() != CREDIT_) {
                // pushes a value to extraCredit queue of the co-bridge
                // the value is flitPossible, which is ser ratio of the data
                // flit
                // these flits are necessarily the same width, even though
                // credit flits
                // are emphemrel (have no size/data). We *know* a credit will
                // be sent
                // after data flit transmission to free up the queue
                // this sends the ratio to coBridge, which is the credit link
                // for this
                // data link so that the credit can be fake serialized despite
                // lacking a body
                printf("***** Serializer CoBridge Neutralize, val: %d\r\n",
                                flitPossible);
                coBridge->neutralize(vc, flitPossible);
            }
            // Delete this flit, new flit is sent in any case
            delete t_flit;
        }
        return;
    }

    // If only CDC is enabled schedule it
    scheduleFlit(t_flit, Cycles(0));
}

void
NetworkBridge::pawsFlitisizeAndSend(flit *t_flit)
{
    // Serialize-Deserialize only if it is enabled
    if (enSerDes) {
        // Calculate the target-width
        int target_width = bitWidth;
        int cur_width = nLink->bitWidth;
        if (mType == Enums::OBJECT_LINK) {
            target_width = nLink->bitWidth;
            cur_width = bitWidth;
        }

        DPRINTF(RubyNetwork, "Target width: %d Current: %d\n",
            target_width, cur_width);
        assert(target_width != cur_width);

        int vc = t_flit->get_vc();


        if (target_width > cur_width) {
            // Deserialize
            // This deserializer combines flits from the
            // same message together
            int num_flits = 0;
            int flitPossible = 0;
            if (t_flit->get_type() == CREDIT_) {
                lenBuffer[vc]++;
                assert(extraCredit[vc].front());
                printf("***** Deserializer Rx Credit, EC front val: %d\r\n",
                                extraCredit[vc].front());
                if (lenBuffer[vc] == extraCredit[vc].front()) {
                    printf("***** Deser EC==LB, EC front val: %d\r\n",
                                    lenBuffer[vc]);
                    flitPossible = 1;
                    extraCredit[vc].pop();
                    lenBuffer[vc] = 0;
                }
            } else if (t_flit->get_type() == TAIL_ ||
                       t_flit->get_type() == HEAD_TAIL_) {
                // If its the end of packet, then send whatever
                // is available.
                int sizeAvail = (t_flit->msgSize - sizeSent[vc]);
                flitPossible = ceil((float)sizeAvail/(float)target_width);
                assert (flitPossible < 2);
                num_flits = (t_flit->get_id() + 1) - flitsSent[vc];
                // Stop tracking the packet.
                flitsSent[vc] = 0;
                sizeSent[vc] = 0;
            } else {
                // If we are yet to receive the complete packet
                // track the size recieved and flits deserialized.
                int sizeAvail =
                    ((t_flit->get_id() + 1)*cur_width) - sizeSent[vc];
                flitPossible = floor((float)sizeAvail/(float)target_width);
                assert (flitPossible < 2);
                num_flits = (t_flit->get_id() + 1) - flitsSent[vc];
                if (flitPossible) {
                    sizeSent[vc] += target_width;
                    flitsSent[vc] = t_flit->get_id() + 1;
                }
            }

            DPRINTF(RubyNetwork, "Deserialize :%dB -----> %dB "
                " vc:%d\n", cur_width, target_width, vc);

            flit *fl = NULL;
            if (flitPossible) {
                fl = t_flit->deserialize(lenBuffer[vc], num_flits,
                    target_width);
            }

            // Inform the credit serializer about the number
            // of flits that were generated.
            if (t_flit->get_type() != CREDIT_ && fl) {
                printf("***** Deserializer CoBridge Neutralize, val: %d\r\n",
                                num_flits);
                coBridge->neutralize(vc, num_flits);
            }

            // Schedule only if we are done deserializing
            if (fl) {
                DPRINTF(RubyNetwork, "Scheduling a flit\n");
                lenBuffer[vc] = 0;
                scheduleFlit(fl, serDesLatency);
            }
            // Delete this flit, new flit is sent in any case
            delete t_flit;
        } else {
            // Serialize
            DPRINTF(RubyNetwork, "Serializing flit :%d -----> %d "
            "(vc:%d, Original Message Size: %d)\n",
                cur_width, target_width, vc, t_flit->msgSize);

            // a previous serflit is in the cache and wasn't sent
            // we got a new flit, the timer didn't expire
            // we're going to pack this flit into the cached
            // flit
            bool pack = packing_timer_active[vc] || ser_flit_cache[vc] == NULL;

            int flitPossible = 0;
            if (t_flit->get_type() == CREDIT_) {
                // if my understanding is correct teh credit link is distinct
                // from
                // from teh data link and we will force flushes on credit for
                // latency and because there's no speed up based on the garnet
                // model
                assert(!pack);

                // We store the deserialization ratio and then
                // access it when serializing credits in the
                // oppposite direction.
                assert(extraCredit[vc].front());
                flitPossible = extraCredit[vc].front();
                extraCredit[vc].pop();
                printf("***** Serializer Rx Credit, EC front val: %d\r\n",
                                flitPossible);
            } else if (t_flit->get_type() == HEAD_ ||
                    t_flit->get_type() == BODY_) {
                // id is the index into the packet
                // its weird that sizeAvail seems to go up as id goes up, but
                // this is compensated for in (- sizeSent[vc]) which is zeroed
                // out when the tail comes along
                // cur_wdith = fatter link
                // target_width = narrower link
                // flit
                int sizeAvail =
                    ((t_flit->get_id() + 1)*cur_width) - sizeSent[vc];
                flitPossible = floor((float)sizeAvail/(float)target_width);
                if (flitPossible) {
                    sizeSent[vc] += flitPossible*target_width;
                    flitsSent[vc] += flitPossible;
                }
            } else {
                // type = TAIL_ OR HEAD_TAIL_
                int sizeAvail = t_flit->msgSize - sizeSent[vc];
                flitPossible = ceil((float)sizeAvail/(float)target_width);
                sizeSent[vc] = 0;
                flitsSent[vc] = 0;
            }
            assert(flitPossible > 0);

            // Schedule all the flits
            // num_flits could be zero for credits
            // under paws credit links should flush everything immediately
            // no packing
            // CLEAN UP THE t_flit MEMORY BEFORE RETURNING EARLY WATCH THIS
            // LATER AS WELL
            if (t_flit->get_type() == CREDIT_) {
                    for (int i = 0; i < flitPossible; i++) {
                        // Ignore neutralized credits
                        flit *fl = t_flit->serialize(i,
                                        flitPossible, target_width);
                        scheduleFlit(fl, serDesLatency);
                        DPRINTF(RubyNetwork,
                        "Serialized credit flit to flit[%d of %d parts]:"
                        " %s\n", i+1, flitPossible, *fl);
                    }

                    delete t_flit;
                    return;
            }

            assert(t_flit->get_type() != CREDIT_);

            // do paws serialization
            // no credit flits from here on out

            // theres already a partial flit in the cache, lets fill it
            if (pack) {

                // PACK THE CACHED FLIT

                int bytes_consumed = ser_flit_cache[vc]->get_packed_ind() + 1;
                int bytes_slack = target_width - bytes_consumed;

                // correct flit possible
                int sizeAvail = t_flit->msgSize - sizeSent[vc] - bytes_slack;
                int additional_flits = ceil(
                                (float) sizeAvail / (float) target_width);
                int new_slack = target_width % sizeAvail;

                // mark serflit
                ser_flit_cache[vc]->set_packed(true);
                // this may be totally unnecessary
                ser_flit_cache[vc]->set_second_flit(t_flit);
                // send serflit
                scheduleFlit(ser_flit_cache[vc], serDesLatency);
                DPRINTF(RubyNetwork, "Send cached flit after packing %s, %s\n",
                                ser_flit_cache[vc], t_flit);
                // clear the cache
                ser_flit_cache[vc] = NULL;
                // stop the flush timers
                packing_timer_active[vc] = false;
                packing_timer[vc] = -1;

                // SEND NON PACKED INTERMEDIATE FLITS

                // send any serflits not previously packed and not
                // the last flit
                for (int i = 0; i < additional_flits - 1; i++) {
                    // Ignore neutralized credits
                    flit *fl = t_flit->serialize(
                                    i, additional_flits, target_width);
                    fl->set_packed(false);
                    scheduleFlit(fl, serDesLatency);
                    DPRINTF(RubyNetwork, "Serialized to flit[%d of %d parts]:"
                    " %s\n", i+1, additional_flits, *fl);
                }

                // SEND OR CACHE (to pack later) THE LAST FLIT

                flit *fl = t_flit->serialize(
                                flitPossible - 1, flitPossible, target_width);
                fl->set_packed(false);
                if (new_slack == 0) {
                    // perfectly aligned, no packing, send now
                    scheduleFlit(fl, serDesLatency);
                } else {
                    // make opportunity for packing

                    // mark the slack in the flit metadata
                    fl->set_packed_ind(target_width - new_slack - 1);

                    // cache the flit
                    ser_flit_cache[vc] = fl;

                    // start the timer
                    packing_timer_active[vc] = true;
                    packing_timer[vc] = 0;
                }
            } else {
                // nothing in the cache, not packing

                // Schedule all the flits
                // num_flits could be zero for credits
                for (int i = 0; i < flitPossible - 1; i++) {
                    // Ignore neutralized credits
                    flit *fl = t_flit->serialize(
                                    i, flitPossible, target_width);
                    scheduleFlit(fl, serDesLatency);
                    DPRINTF(RubyNetwork, "Serialized to flit[%d of %d parts]:"
                    " %s\n", i+1, flitPossible, *fl);
                }

                // last serflit, check if aligned
                flit *fl = t_flit->serialize(
                                flitPossible - 1, flitPossible, target_width);
                fl->set_packed(false);

                int sizeAvail = t_flit->msgSize - sizeSent[vc];
                int slack = target_width % sizeAvail;
                if (slack == 0) {
                    scheduleFlit(fl, serDesLatency);
                } else {
                    fl->set_packed_ind(target_width - slack - 1);

                    ser_flit_cache[vc] = fl;

                    packing_timer_active[vc] = true;
                    packing_timer[vc] = 0;
                }
            }

            // pushes a value to extraCredit queue of the co-bridge
            // the value is flitPossible, which is ser ratio of the data flit
            // these flits are necessarily the same width, even though credit
            // flits
            // are emphemrel (have no size/data). We *know* a credit will be
            // sent
            // after data flit transmission to free up the queue
            // this sends the ratio to coBridge, which is the credit link
            // for this
            // data link so that the credit can be fake serialized despite
            // lacking
            // a body
            // this is no longer valid for paws since the possible count is
            // dynamic
            // based on slack the credits will always be serialized normally
            // and never packed
            int cred_flit_ct = ceil(
                            (float) t_flit->msgSize / (float)target_width);
            printf("***** Serializer CoBridge Neutralize, val: %d\r\n",
                            cred_flit_ct);
            coBridge->neutralize(vc, cred_flit_ct);
            // Delete this flit, new flit is sent in any case
            delete t_flit;
        }
        return;
    }

    // If only CDC is enabled schedule it
    scheduleFlit(t_flit, Cycles(0));
}

void
NetworkBridge::pawsFlushVc(int vc) {
        scheduleFlit(ser_flit_cache[vc], serDesLatency);
        DPRINTF(RubyNetwork, "Timeout flushed cached flit %s\n",
                        ser_flit_cache[vc]);
        // clear the cache
        ser_flit_cache[vc] = NULL;
        // stop the flush timers
        packing_timer_active[vc] = false;
        packing_timer[vc] = -1;
}

void
NetworkBridge::wakeup()
{
    // this can manually wake up the NB every cycle
    // but we've contextualized to the PAWS flush timers
    // scheduleEvent(Cycles(1));
    // printf("***** net bridge wakeup, srcQueue rdy? : %d\r\n",
    // link_srcQueue->isReady(curTick()));
    flit *t_flit;

    const int policy = 0;
    const int PACKING_TIMER_THRESH = 5;
    const int NUM_VCS = 1;

    switch (policy) {
        case 1: {
            // if we have a valid packet, do paws
            if (link_srcQueue->isReady(curTick())) {
                t_flit = link_srcQueue->getTopFlit();
                pawsFlitisizeAndSend(t_flit);
            }

            // check timers after to make sure the flags are active
            // when scheduling the wakeup with the MEQ
            bool hasScheduledTimerEvent = false;
            // do a quick check for vcs who need a flush
            for (int vc = 0; vc < NUM_VCS; vc++) {
                if (packing_timer_active[vc]) {
                    packing_timer[vc]++;
                    if (packing_timer[vc] > PACKING_TIMER_THRESH) {
                        // this de-asserts the timer flag
                        pawsFlushVc(vc);
                    }

                    // we have an active timer, we need to call this link
                    // from the global event queue to manage flushes
                    if (packing_timer_active[vc] && !hasScheduledTimerEvent) {
                        scheduleEvent(Cycles(1));
                        hasScheduledTimerEvent = true;
                    }
                }
            }

            return;
            }
        case 0:
        default: {
            if (link_srcQueue->isReady(curTick())) {
                t_flit = link_srcQueue->getTopFlit();
                DPRINTF(RubyNetwork, "Recieved flit %s\n", *t_flit);
                flitisizeAndSend(t_flit);
                return;
            }
        }
    }
    // this only holds if the only event queue wakeup
    // is out of the crossbar
    // we need to schedule more often during the timer activity
    // to flush paws, so this no longer holds
    // assert(!link_srcQueue->getSize());
}
