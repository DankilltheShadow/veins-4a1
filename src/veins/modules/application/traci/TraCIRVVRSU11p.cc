//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "veins/modules/application/traci/TraCIRVVRSU11p.h"
#include <iostream>
#include <fstream>

using Veins::AnnotationManagerAccess;

Define_Module(TraCIRVVRSU11p);

void TraCIRVVRSU11p::Statistics::initialize()
{
    numCH.setName("Number_CH");
    numON.setName("Number_ON");
    numFN.setName("Number_FN");
    numCHLost.setName("Number_CH_Losts");
    meanClusterSize.setName("Mean_Cluster_Size");
    stddevCluster.setName("StdDev_Cluster_Size");
    varianceCluster.setName("Variance_Cluster_Size");
    meanNeighbors.setName("Mean_Neighbors");
    stddevNeighbors.setName("StdDev_Neighbors");
    varianceNeighbors.setName("Variance_Neighbors");
    // CH
    totalCHUtility.setName("Total_CH_Utility");
    meanCHUtility.setName("Mean_CH_Utility");
    stddevCHUtility.setName("StdDev_CH_Utility");
    varianceCHUtility.setName("Variance_CH_Utility");
    totalExpCHUtility.setName("Total_exp_CH_Utility");
    meanExpCHUtility.setName("Mean_exp_CH_Utility");
    stddevExpCHUtility.setName("StdDev_exp_CH_Utility");
    varianceExpCHUtility.setName("Variance_exp_CH_Utility");
    // ON
    totalONUtility.setName("Total_ON_Utility");
    meanONUtility.setName("Mean_ON_Utility");
    stddevONUtility.setName("StdDev_ON_Utility");
    varianceONUtility.setName("Variance_ON_Utility");
    totalExpONUtility.setName("Total_exp_ON_Utility");
    meanExpONUtility.setName("Mean_exp_ON_Utility");
    stddevExpONUtility.setName("StdDev_exp_ON_Utility");
    varianceExpONUtility.setName("Variance_exp_ON_Utility");
}

void TraCIRVVRSU11p::initialize(int stage) {
	BaseWaveApplLayer::initialize(stage);
	if (stage == 0) {
		mobi = dynamic_cast<BaseMobility*> (getParentModule()->getSubmodule("mobility"));
		ASSERT(mobi);
		annotations = AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);
		sentMessage = false;
		startMatching = new cMessage("Start!", SEND_MATCH);
		statistics.initialize();
		scheduleAt(simTime() + par("startMatching").doubleValue(), startMatching);
	}
}

void TraCIRVVRSU11p::onBeacon(WaveShortMessage* wsm) {

}

void TraCIRVVRSU11p::onData(WaveShortMessage* wsm) {
	findHost()->getDisplayString().updateWith("r=16,green");

	annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobi->getCurrentPosition(), "blue"));

	if (!sentMessage) sendMessage(wsm->getWsmData());
}

void TraCIRVVRSU11p::sendMessage(std::string blockedRoadId) {
	sentMessage = true;
	t_channel channel = dataOnSch ? type_SCH : type_CCH;
	WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
	wsm->setWsmData(blockedRoadId.c_str());
	sendWSM(wsm);
}

void TraCIRVVRSU11p::sendWSM(WaveShortMessage* wsm) {
	sendDelayedDown(wsm,individualOffset);
}

void TraCIRVVRSU11p::handleLowerMsg(cMessage* msg) {

    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    ASSERT(wsm);

    if (std::string(wsm->getName()) == "Hello") {
        updatePreferenceList(wsm);
    } else if (std::string(wsm->getName()) == "beacon") {
        onBeacon(wsm);
    }
    else if (std::string(wsm->getName()) == "data") {
        onData(wsm);
    }
    else {
        DBG << "unknown message (" << wsm->getName() << ")  received\n";
    }
    delete(msg);
}

void TraCIRVVRSU11p::updatePreferenceList(WaveShortMessage* wsm) {
    int id = wsm->getSenderAddress();

    PrefMap::iterator it = PrefCHLists.find(id);
    if(it != PrefCHLists.end()){
        PrefCHLists.erase(it);
    }
    it = PrefONLists.find(id);
    if(it != PrefONLists.end()){
        PrefONLists.erase(it);
    }

    PrefList list;
    for (size_t i=0; i<wsm->getPrefListArraySize(); ++i){
        list.push_back(wsm->getPrefList(i));
    }
    if(std::string(wsm->getSenderState())=="CH"){
        PrefCHLists.insert(std::pair<int, std::vector<int>> (id, list));
        CHcapacity.insert(std::pair<int, int> (id, wsm->getCapacity()));
    }else{
        PrefONLists.insert(std::pair<int, std::vector<int>> (id, list));
    }
    nodesCoord[id] = wsm->getSenderPos();
}

void TraCIRVVRSU11p::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {
        case SEND_MATCH: {
            Matched = RVV(Matched);
            if(simTime()>=simulation.getWarmupPeriod()){
                orgStatistic();
            }
            scheduleAt(simTime() + par("startMatching").doubleValue(), startMatching);
            break;
        }
        default: {
            if (msg)
                BaseWaveApplLayer::handleSelfMsg(msg);
            break;
        }
    }
}

double TraCIRVVRSU11p::calcUtility(double sqrD){
    double sigmaQ=FWMath::dBm2mW(-110); //converto i dBm in mW per sigma quadro
    double pDivSigma = 10/sigmaQ; // 10 mW
    double w = 20; //MHZ
    double g = 1/sqrD;

    return(w*log(1+(pDivSigma*g)));
}

/*void TraCIRVVRSU11p::adjustPrefList() {
    for(auto p = PrefCHLists.begin(); p != PrefCHLists.end(); ++p){
        PrefList list = p->second;
        PrefList goodList;
        for(size_t jt=0;jt<list.size();jt++){
            const int ON = list[jt];
            if (PrefONLists.find(ON) != PrefONLists.end()) {
               goodList.push_back(ON);
            }
        }
        p->second = goodList;
    }
    for(auto p = PrefONLists.begin(); p != PrefONLists.end(); ++p){
        PrefList list = p->second;
        PrefList goodList;
        for(size_t jt=0;jt<list.size();jt++){
            const int CH = list[jt];
            if (PrefCHLists.find(CH) != PrefCHLists.end()) {
                goodList.push_back(CH);
            }
        }
        p->second = goodList;
    }
}*/

void TraCIRVVRSU11p::orgStatistic() {
    cStdDev statsBasic;
    cStdDev statsNeighbors;
    cStdDev statsCHutility;
    cStdDev statsONutility;
    cStdDev statsExpCHutility;
    cStdDev statsExpONutility;

    for(auto const& p :PrefCHLists){
        const int CH = p.first;
        const PrefList prefCH = p.second;
        auto ret = Matched.equal_range(CH);
        double numerator = 0;
        double denominator = 0;
        for(auto p = ret.first; p!=ret.second; ++p){
            const int ON = p->second;
            numerator += calcUtility(nodesCoord[CH].distance(nodesCoord[ON]));
            denominator += 10;
        }
        if(denominator!=0){
            double UCH;
            UCH = numerator/denominator;
            statsCHutility.collect(UCH);
            statsBasic.collect(denominator/10);
        }
        numerator=0;
        denominator=0;
        for(auto const& it : prefCH){
            const int ON = it;
            numerator += calcUtility(nodesCoord[CH].distance(nodesCoord[ON]));
            denominator += 10;
        }
        if(denominator!=0){
            double UCH;
            UCH = numerator/denominator;
            statsExpCHutility.collect(UCH);
            statsNeighbors.collect(denominator/10);
        }
    }
    for(auto const& p :PrefONLists){
        const int ON = p.first;
        const PrefList prefON = p.second;
        double value = 0;
        double N = 0;
        for(auto const& it : prefON){
            const int CH = it;
            value += calcUtility(nodesCoord[CH].distance(nodesCoord[ON]))/10;
            N++;
        }
        if(N!=0){
            double UON;
            UON = value/N;
            statsExpONutility.collect(UON);
            statsNeighbors.collect(N);
        }
        value = 0;
        N = 0;
        for(auto const& it : Matched){
            const int CH = it.first;
            if(it.second == ON){
                value = calcUtility(nodesCoord[CH].distance(nodesCoord[ON]))/10;
                N = 1;
                break;
            }
        }
        if(N!=0){
            double UON;
            UON = value/N;
            statsONutility.collect(UON);
        }
    }
    //cStdDev statsBasic;
    //cStdDev statsNeighbors;
    //cStdDev statsCHutility;
    //cStdDev statsONutility;
    //cStdDev statsExpCHutility;
    //cStdDev statsExpONutility;
    //Basic stats
    statistics.numCH.record(statsBasic.getCount());
    statistics.numON.record(statsBasic.getSum());
    statistics.numFN.record((PrefCHLists.size()+PrefONLists.size())-(statsBasic.getSum()+statsBasic.getCount()));
    statistics.numCHLost.record(PrefCHLists.size()-statsBasic.getCount());
    statistics.meanClusterSize.record(statsBasic.getMean());
    statistics.stddevCluster.record(statsBasic.getStddev());
    statistics.varianceCluster.record(statsBasic.getVariance());
    //Neighbors stats
    statistics.meanNeighbors.record(statsNeighbors.getMean());
    statistics.stddevNeighbors.record(statsNeighbors.getStddev());
    statistics.varianceNeighbors.record(statsNeighbors.getVariance());
    //CH stats
    statistics.totalCHUtility.record(statsCHutility.getSum());
    statistics.meanCHUtility.record(statsCHutility.getMean());
    statistics.stddevCHUtility.record(statsCHutility.getStddev());
    statistics.varianceCHUtility.record(statsCHutility.getVariance());
    statistics.totalExpCHUtility.record(statsExpCHutility.getSum());
    statistics.meanExpCHUtility.record(statsExpCHutility.getMean());
    statistics.stddevExpCHUtility.record(statsExpCHutility.getStddev());
    statistics.varianceExpCHUtility.record(statsExpCHutility.getVariance());
    //ON stats
    statistics.totalONUtility.record(statsONutility.getSum());
    statistics.meanONUtility.record(statsONutility.getMean());
    statistics.stddevONUtility.record(statsONutility.getStddev());
    statistics.varianceONUtility.record(statsONutility.getVariance());
    statistics.totalExpONUtility.record(statsExpONutility.getSum());
    statistics.meanExpONUtility.record(statsExpONutility.getMean());
    statistics.stddevExpONUtility.record(statsExpONutility.getStddev());
    statistics.varianceExpONUtility.record(statsExpONutility.getVariance());
}
//////////////////////////////////////////////////////////////////////////////
///////////////////RVV Algorithm/////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
Matching TraCIRVVRSU11p::RVV(Matching Mz){
    //adjustPrefList();
    Matching tempM = Mz;
    std::map<int, int> S;
    ////////////////////////////////////////////////////////////////
    //Scrive file di matching
    std::ofstream myfile;
    myfile.open ("matching.txt");
    myfile<<simTime().dbl()<<"\n";
    myfile<<"{";
    for(auto iter=CHcapacity.begin(); iter!=CHcapacity.end(); ++iter){
        myfile<<"{"<<(*iter).first<<","<<(*iter).second<<"},";
    }
    myfile<<"}\n";
    myfile<<"Match\n";
    myfile<<"{";
    for(auto iter=tempM.begin(); iter!=tempM.end(); ++iter){
        myfile<<"{"<<(*iter).first<<","<<(*iter).second<<"},";
    }
    myfile<<"}\n";
    myfile<<"CHs\n";
    myfile<<"{";
    for(auto& p : PrefCHLists){
        myfile<<"{"<<p.first<<",{";
        const PrefList list = p.second;
        for(size_t jt=0;jt<list.size();jt++){
            if(jt==list.size()-1)
                myfile<<list[jt];
            else
                myfile<<list[jt]<<",";
        }
        myfile<<"}},";
    }
    myfile<<"}\n";
    myfile<<"ONs\n";
    myfile<<"{";
    for(auto& p : PrefONLists){
        myfile<<"{"<<p.first<<",{";
        const PrefList list = p.second;
        for(size_t jt=0;jt<list.size();jt++){
            if(jt==list.size()-1)
                myfile<<list[jt];
            else
                myfile<<list[jt]<<",";
        }
        myfile<<"}},";
    }
    myfile<<"}\n";
    myfile.close();
 //   if(PrefCHLists.size()!=0 && PrefONLists.size()!=0){
    Matching bp = foundBP(tempM, PrefCHLists, PrefONLists);
    int t=0;
    while (!bp.empty()) {
        t++;
        bool found = false;
        for(auto const& p : bp){
            const int CH = p.first;
            const int ON = p.second;
            if(S.find(CH)!=S.end() && S.find(ON)==S.end()){
                found = add(ON, 0, S, tempM); //1=CH,0=ON
                break;
            }else if(S.find(ON)!=S.end() && S.find(CH)==S.end()){
                found = add(CH, 1, S, tempM); //1=CH,0=ON
                break;
            }
        }
        if(!found){
            satisfy(bp.begin()->first,bp.begin()->second, S, tempM);
        }
        bp.clear();
        bp = foundBP(tempM, PrefCHLists, PrefONLists);
    }
/*    }else{
        tempM.clear();
    }*/

    return tempM;

}

Matching TraCIRVVRSU11p::foundBP(Matching m, PrefMap fPrefCHLists, PrefMap fPrefONLists){
    Matching BP;
    for(auto const& p : fPrefCHLists){
        const int CH = p.first;
        const PrefList CHpref = p.second;
        std::pair<Matching::iterator, Matching::iterator> ret;
        ret = m.equal_range(CH);
        PrefList ONmatched;
        for( Matching::iterator it = ret.first; it != ret.second; it++ ){
            ONmatched.push_back(it->second);
        }
        size_t j = 0;
        for( int i = 0; i < CHcapacity[CH]; i++ ){
            if(j < CHpref.size()){
                const int ON = CHpref[j];
                PrefList ONpref;
                bool found = false;
                bool foundCH = false;
                if(fPrefONLists.find(ON)!=fPrefONLists.end())
                    ONpref = fPrefONLists.find(ON)->second;
                else
                    ONpref.clear();
                for(size_t iter = 0; iter < ONpref.size(); iter++){
                    if(ONpref[iter]==CH)
                        foundCH=true;
                    break;
                }
                for( size_t k = 0; k < ONmatched.size(); k++ ){
                    if( ON == ONmatched[k] ){
                        found=true;
                        break;
                    }
                }
                if(!found && foundCH){
                    bool insertBP = true;
                    for( Matching::iterator it = m.begin(); it != m.end(); it++ ){
                        if(it->second == ON){
                            const int onCH = it->first;
                            for(size_t iter = 0; iter < ONpref.size(); iter++){
                                if(ONpref[iter]==CH){
                                    break;
                                }
                                if(ONpref[iter]==onCH){
                                    insertBP = false;
                                    i--;
                                    break;
                                }
                            }
                            break;
                        }
                    }
                    if(insertBP){
                        BP.insert(std::pair<int, int>(CH,ON));
                    }
                }
                j++;
            }
        }
    }
    return BP;
}

bool TraCIRVVRSU11p::add(int a, int role, std::map<int, int> &S, Matching &m){
    if(role==0){
        int ON = a;
        for (auto it = m.begin(); it != m.end(); it++){
            if (it->second == ON){
                m.erase(it);
                break;
            }
        }
        S[ON]=1;
        Matching aBps = blockingAgent(ON, S, m);
        while(!aBps.empty()){
            Matching::iterator bestIt = aBps.begin();
            const int inCH = bestIt->first;
            const int inON = bestIt->second;
            if(CHcapacity[inCH]<=m.count(inCH)){
                Matching::iterator worst;
                int iWorst=-1;
                std::pair <Matching::iterator, Matching::iterator> ret;
                ret = m.equal_range(inCH);
                PrefList CHpref;
                if(PrefCHLists.find(inCH)!=PrefCHLists.end())
                    CHpref = PrefCHLists.find(inCH)->second;
                else
                    CHpref.clear();
                for( Matching::iterator it = ret.first; it!=ret.second; it++ ){
                    const int tON = it->second;
                    for(int jt = 0; jt < CHpref.size(); jt++){
                        if(CHpref[jt] == tON){
                            if(iWorst < jt){
                                iWorst = jt;
                                worst = it;
                                break;
                            }else {
                                break;
                            }
                        }else if(jt == CHpref.size()-1){
                            iWorst = -1;
                        }
                    }
                    if(iWorst==-1){
                        worst = it;
                        break;
                    }
                }
                ON = worst->second;
                m.erase(worst);
            }
            m.insert(std::pair<int,int>(inCH,inON));
            aBps = blockingAgent(ON, S, m);
        }
    }else{
        int CH = a;
        if(CHcapacity[CH]<=m.count(CH)){
            Matching::iterator worst;
            int iWorst=-1;
            std::pair <Matching::iterator, Matching::iterator> ret;
            ret = m.equal_range(CH);
            PrefList CHpref;
            if(PrefCHLists.find(CH)!=PrefCHLists.end())
                CHpref = PrefCHLists.find(CH)->second;
            else
                CHpref.clear();
            for( Matching::iterator it = ret.first; it!=ret.second; it++ ){
                const int tON = it->second;
                for(int jt = 0; jt < CHpref.size(); jt++){
                    if(CHpref[jt] == tON){
                        if(iWorst < jt){
                            iWorst = jt;
                            worst = it;
                            break;
                        }else {
                            break;
                        }
                    }else if(jt==CHpref.size()-1){
                        iWorst = -1;
                    }
                }
                if(iWorst==-1){
                    worst = it;
                    break;
                }
            }
            m.erase(worst);
        }
        S[CH]=1;
        Matching aBps = blockingAgent(CH, S, m);
        while(!aBps.empty()){
            Matching::iterator bestIt = aBps.begin();
            const int inCH = bestIt->first;
            const int inON = bestIt->second;
            for (auto it = m.begin(); it != m.end(); it++){
                if (it->second == inON){
                    CH = it->first;
                    m.erase(it);
                    break;
                }
            }
            m.insert(std::pair<int,int>(inCH,inON));
            aBps = blockingAgent(CH, S, m);
        }
    }
    return true;
}

Matching TraCIRVVRSU11p::blockingAgent(int agent, std::map<int, int> S, Matching m){
    Matching bestbps;
    for (auto it = m.begin(); it != m.end(); it++){
        if (S.find(it->first) == S.end() || S.find(it->second) == S.end()){
            it = m.erase(it);
        }
    }
    PrefMap tempPrefCHLists = PrefCHLists;
    PrefMap tempPrefONLists = PrefONLists;
    PrefMap::iterator tempListit = tempPrefCHLists.begin();
    while (tempListit != tempPrefCHLists.end()){
        if (S.find(tempListit->first) == S.end()){
            tempListit = tempPrefCHLists.erase(tempListit);
        }else{
            const int CH = tempListit->first;
            PrefList list = tempListit->second;
            PrefList goodList;
            for(size_t jt=0;jt<list.size();jt++){
                const int ON = list[jt];
                if (S.find(ON) != S.end()) {
                    goodList.push_back(ON);
                }
            }
            tempPrefCHLists[CH] = goodList;
            ++tempListit;
        }
    }
    tempListit = tempPrefONLists.begin();
    while (tempListit != tempPrefONLists.end()){
        if (S.find(tempListit->first) == S.end()){
            tempListit = tempPrefONLists.erase(tempListit);
        }else{
            const int ON = tempListit->first;
            PrefList list = tempListit->second;
            PrefList goodList;
            for(size_t jt=0;jt<list.size();jt++){
                const int CH = list[jt];
                if (S.find(CH) != S.end()) {
                    goodList.push_back(CH);
                }
            }
            tempPrefONLists[ON] = goodList;
            ++tempListit;
        }
    }
    Matching bpM = foundBP(m, tempPrefCHLists, tempPrefONLists);
    if(!bpM.empty()){
        PrefList list;
        if(PrefCHLists.find(agent)!=PrefCHLists.end()){
            list = PrefCHLists.find(agent)->second;
        }else if(PrefONLists.find(agent)!=PrefONLists.end()){
            list = PrefONLists.find(agent)->second;
        }
        for(size_t i=0; i<list.size();i++){
            if(bestbps.size()>0){
                break;
            }
            const int best = list[i];
            for (auto it = bpM.begin(); it != bpM.end(); it++){
                if((it->first==best && it->second==agent) || (it->second==best && it->first==agent)){
                    bestbps.insert(std::pair<int,int>(it->first,it->second));
                    break;
                }
            }
        }
    }
    return bestbps;
}

void TraCIRVVRSU11p::satisfy(int CH, int ON, std::map<int, int> &S, Matching &m ){
    S[ON]=1;
    S[CH]=1;
    for (auto it = m.begin(); it != m.end(); it++){
        if (it->second == ON){
            m.erase(it);
            break;
        }
    }
    if(CHcapacity[CH]<=m.count(CH)){
        Matching::iterator worst;
        int iWorst=-1;
        std::pair <Matching::iterator, Matching::iterator> ret;
        ret = m.equal_range(CH);
        PrefList CHpref;
        if(PrefCHLists.find(CH)!=PrefCHLists.end())
            CHpref = PrefCHLists.find(CH)->second;
        else
            CHpref.clear();
        for( Matching::iterator it = ret.first; it!=ret.second; it++ ){
            const int tON = it->second;
            for(int jt = 0; jt < CHpref.size(); jt++){
                if(CHpref[jt] == tON){
                    if(iWorst < jt){
                        iWorst = jt;
                        worst = it;
                        break;
                    }else {
                        break;
                    }
                }else if(jt==CHpref.size()-1){
                    iWorst = -1;
                }
            }
            if(iWorst==-1){
                worst = it;
                break;
            }
        }
        m.erase(worst);
    }
    m.insert(std::pair<int,int>(CH,ON));
}
