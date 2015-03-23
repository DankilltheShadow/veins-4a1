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
    numCH=0;
    numON=0;
    numFN=0;
    meanCluster=0;
    sigmaCluster=0;
    CHutility=0;
    meanCHutility=0;
    sigmaCHutility=0;
    expCHutility=0;
    meanexpCHutility=0;
    sigmaexpCHutility=0;
    diffCHutility=0;
    meandiffCHutility=0;
    sigmadiffCHutility=0;
}

void TraCIRVVRSU11p::Statistics::recordScalars(cSimpleModule& module)
{
    module.recordScalar("NumberofCH", numCH);
    module.recordScalar("NumberofON", numON);
    module.recordScalar("NumberofFN", numFN);
    module.recordScalar("meanClusterNodes", meanCluster);
    module.recordScalar("varClusterNodes", sigmaCluster);
    module.recordScalar("sumCHutility", CHutility);
    module.recordScalar("meanCHutility", meanCHutility);
    module.recordScalar("varCHutility", sigmaCHutility);
    module.recordScalar("sumDiffCHutility", diffCHutility);
    module.recordScalar("meanDiffCHutility", meandiffCHutility);
    module.recordScalar("varDiffCHutility", sigmadiffCHutility);
    module.recordScalar("expectedCHutility", expCHutility);
    module.recordScalar("meanExpectedCHutility", meanexpCHutility);
    module.recordScalar("varExpectedCHutility", sigmaexpCHutility);
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

void TraCIRVVRSU11p::orgStatistic() {

    statistics.numON = Matched.size();
    statistics.numFN -= Matched.size();
    statistics.meanCluster = (statistics.numCH!=0) ? statistics.numON /statistics.numCH : statistics.numON;

    double var = 0;
    for(auto const& p : PrefCHLists){
        const int CH = p.first;
        double numONCluster = Matched.count(CH);
        var += (numONCluster-statistics.meanCluster)*(numONCluster-statistics.meanCluster);
        std::pair <Matching::iterator, Matching::iterator> ret;
        ret = Matched.equal_range(CH);
        double sumU = 0;
        for( Matching::iterator it = ret.first; it != ret.second; it++ ){
            const int ON = it->second;
            double sqrDistance = nodesCoord[CH].sqrdist(nodesCoord[ON]);
            sumU += calcUtility(sqrDistance);
        }
        double sumexpU = 0;
        PrefList preflistCH = p.second;
        int countON = 0;
        for( size_t itpCH = 0; itpCH < preflistCH.size(); itpCH++ ){
            const int ON = preflistCH[itpCH];
            PrefMap::iterator it;
            it = PrefCHLists.find(ON);
            if(it==PrefCHLists.end())/* && CHcapacity[CH]<countON)*/{
                double sqrDistance = nodesCoord[CH].sqrdist(nodesCoord[ON]);
                sumexpU += calcUtility(sqrDistance);
                countON++;
            }
        }
        double oneCHutility = (numONCluster!= 0) ? sumU/(10*numONCluster) : 0;
        statistics.CHutility += oneCHutility;
        double oneexpCHutility = (countON!= 0) ? sumexpU/(10*countON) : 0;
        statistics.expCHutility += oneexpCHutility;
        statistics.diffCHutility += oneexpCHutility-oneCHutility;
    }
    statistics.meanCHutility = (statistics.numCH!=0) ? statistics.CHutility/statistics.numCH : statistics.CHutility;
    statistics.meanexpCHutility = (statistics.numCH!=0) ? statistics.expCHutility/statistics.numCH : statistics.expCHutility;
    statistics.meandiffCHutility = (statistics.numCH!=0) ? statistics.diffCHutility/statistics.numCH : statistics.diffCHutility;

    /////////////////////////////////////////////////
    for(auto const& p : PrefCHLists){
        const int CH = p.first;
        double numONCluster = Matched.count(CH);
        std::pair <Matching::iterator, Matching::iterator> ret;
        ret = Matched.equal_range(CH);
        double sumU = 0;
        for( Matching::iterator it = ret.first; it != ret.second; it++ ){
            const int ON = it->second;
            double sqrDistance = nodesCoord[CH].sqrdist(nodesCoord[ON]);
            sumU += calcUtility(sqrDistance);
        }
        double sumexpU = 0;
        PrefList preflistCH = p.second;
        int countON = 0;
        for( size_t itpCH = 0; itpCH < preflistCH.size(); itpCH++ ){
            const int ON = preflistCH[itpCH];
            PrefMap::iterator it;
            it = PrefCHLists.find(ON);
            if(it==PrefCHLists.end()){
                double sqrDistance = nodesCoord[CH].sqrdist(nodesCoord[ON]);
                sumexpU += calcUtility(sqrDistance);
                countON++;
            }
        }
        double val = (numONCluster!= 0) ? sumU/(10*numONCluster) : 0;
        statistics.sigmaCHutility += (val - statistics.meanCHutility)*(val - statistics.meanCHutility);
        double valE = (countON!= 0) ? sumexpU/(10*countON) : 0;
        statistics.sigmaexpCHutility += (valE - statistics.meanexpCHutility)*(valE -statistics.meanexpCHutility);
        statistics.sigmadiffCHutility += ((valE-val)-statistics.meandiffCHutility)*((valE-val)-statistics.meandiffCHutility);
    }
    statistics.sigmaCHutility = (statistics.numCH!=0) ? sqrt(statistics.sigmaCHutility/statistics.numCH) : sqrt(statistics.sigmaCHutility);
    statistics.sigmaexpCHutility = (statistics.numCH!=0) ? sqrt(statistics.sigmaexpCHutility/statistics.numCH) : sqrt(statistics.sigmaexpCHutility/1);
    statistics.sigmadiffCHutility = (statistics.numCH!=0) ? sqrt(statistics.sigmadiffCHutility/statistics.numCH) : sqrt(statistics.sigmadiffCHutility/1);
    statistics.sigmaCluster =  (statistics.numCH!=0) ? sqrt(var / statistics.numCH) : sqrt(var / 1);
    //

    ////////////////////////////////////////////////////////////////
    //Scrive file di matching
    /*
    std::ofstream myfile;
    myfile.open ("matching.txt");
    Matching::iterator iter;
    for (iter=Matched.begin(); iter!=Matched.end(); ++iter)
    {
        myfile<<(*iter).first<<"->"<<(*iter).second<<"\n";
    }
    myfile.close();*/
    statistics.recordScalars(*this);
}
//////////////////////////////////////////////////////////////////////////////
///////////////////RVV Algorithm/////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
Matching TraCIRVVRSU11p::RVV(Matching Mz){
    Matching tempM = Mz;
    std::map<int, int> S;
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
            }else if(S.find(ON)!=S.end() && S.find(CH)==S.end()){
                found = add(CH, 1, S, tempM); //1=CH,0=ON
            }
        }
        if(!found){
            satisfy(bp.begin()->first,bp.begin()->second, S, tempM);
        }
        bp.clear();
        bp = foundBP(tempM, PrefCHLists, PrefONLists);
    }

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
                bool found = false;
                for( size_t k = 0; k < ONmatched.size(); k++ ){
                    if( ON == ONmatched[k] ){
                        found=true;
                        break;
                    }
                }
                if(!found){
                    bool insertBP = true;
                    for( Matching::iterator it = m.begin(); it != m.end(); it++ ){
                        if(it->second == ON){
                            const int onCH = it->first;
                            PrefList ONpref = fPrefONLists.find(ON)->second;
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
                PrefList CHpref = PrefCHLists.find(inCH)->second;
                for( Matching::iterator it = ret.first; it!=ret.second; it++ ){
                    const int tON = it->second;
                    for(size_t jt = 0; jt < CHpref.size(); jt++){
                        if(CHpref[jt] == tON){
                            if(iWorst < jt){
                                iWorst = jt;
                                worst = it;
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
            PrefList CHpref = PrefCHLists.find(CH)->second;
            for( Matching::iterator it = ret.first; it!=ret.second; it++ ){
                const int tON = it->second;
                for(size_t jt = 0; jt < CHpref.size(); jt++){
                    if(CHpref[jt] == tON){
                        if(iWorst < jt){
                            iWorst = jt;
                            worst = it;
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
            PrefList &list = tempListit->second;
            PrefList::iterator jt = list.begin();
            for(size_t jt=0;jt<list.size();jt++){
                if (S.find(list[jt]) == S.end()) {
                    list.erase(list.begin()+jt);
                }
            }
            ++tempListit;
        }
    }
    tempListit = tempPrefONLists.begin();
    while (tempListit != tempPrefONLists.end()){
        if (S.find(tempListit->first) == S.end()){
            tempListit = tempPrefONLists.erase(tempListit);
        }else{
            PrefList &list = tempListit->second;
            for(size_t jt=0;jt<list.size();jt++){
                if (S.find(list[jt]) == S.end()) {
                    list.erase(list.begin()+jt);
                }
            }
            ++tempListit;
        }
    }
    Matching bpM = foundBP(m, tempPrefCHLists, tempPrefONLists);
    if(!bpM.empty()){
        PrefMap::iterator iter = (PrefCHLists.find(agent)!=PrefCHLists.end())?PrefCHLists.find(agent):PrefONLists.find(agent);
        PrefList list = iter->second;
        for(size_t i=0; i<list.size();i++){
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
        PrefList CHpref = PrefCHLists.find(CH)->second;
        for( Matching::iterator it = ret.first; it!=ret.second; it++ ){
            const int tON = it->second;
            for(size_t jt = 0; jt < CHpref.size(); jt++){
                if(CHpref[jt] == tON){
                    if(iWorst < jt){
                        iWorst = jt;
                        worst = it;
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
