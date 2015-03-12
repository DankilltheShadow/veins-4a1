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

    if (std::string(wsm->getName()) == "Plist") {
        onPreferenceList(wsm);
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

void TraCIRVVRSU11p::onPreferenceList(WaveShortMessage* wsm) {
    int id = wsm->getSenderAddress();
    PrefList list;

    for (size_t i=0; i<wsm->getPrefListArraySize(); ++i){
        list.push_back(wsm->getPrefList(i));
    }
    if(std::string(wsm->getSenderState())=="CH"){
        PrefCHLists.insert(std::pair<int, std::vector<int>> (id, list));
        CHcapacity.insert(std::pair<int, int> (id, wsm->getCapacity()));
        statistics.numCH++;
    }else{
        PrefONLists.insert(std::pair<int, std::vector<int>> (id, list));
        statistics.numFN++;
    }
    nodesCoord.insert(std::pair<int, Coord> (id, wsm->getSenderPos()));
}

/*
 Si fermano i veicoli a circa 243. ultimo Hello a circa 263. A 265 circa il primo Plist. <<<280 fine Plist. 82 Nodi.*/
void TraCIRVVRSU11p::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {
        case SEND_MATCH: {
            launchMatching();
            break;
        }
        default: {
            if (msg)
                BaseWaveApplLayer::handleSelfMsg(msg);
            break;
        }
    }
}

void TraCIRVVRSU11p::launchMatching() {

    PrefList ONunmatched;
    PrefMap PrefONListsTEMP = PrefONLists;
    for(auto const& iter : PrefONLists){
        ONunmatched.push_back(iter.first);
    }
    while(!ONunmatched.empty()){
        for(size_t it = 0; it<ONunmatched.size(); it++){
            int ON = ONunmatched[it];
            const PrefList &preflist = PrefONListsTEMP[ON];
            if(!preflist.empty()){
                const int &pCH = *preflist.begin();
                if(PrefCHLists.find(pCH)!=PrefCHLists.end()){
                    Matched.insert(std::pair<int,int>(pCH, ON));
                }else{
                    it--;
                }
                PrefONListsTEMP[ON].erase(PrefONListsTEMP[ON].begin());
            }else{
                ONunmatched.erase(ONunmatched.begin()+it);
            }
        }
        for(auto const& p : PrefCHLists){
            const int CH = p.first;
            std::pair <Matching::iterator, Matching::iterator> ret;
            ret = Matched.equal_range(CH);
            int CountON = Matched.count(CH);
            if( CountON > CHcapacity[CH] ){
                PrefList preflistCH = p.second;
                std::map<int, Matching::iterator> Order;
                int size = preflistCH.size();
                for( Matching::iterator it = ret.first; it != ret.second; it++ ){
                    bool found=false;
                    for( size_t itpCH = 0; itpCH < preflistCH.size(); itpCH++ ){
                        if( it->second == preflistCH[itpCH] ){
                            Order[itpCH] = it;
                            found = true;
                            break;
                        }
                    }
                    if(!found){
                        Order[size] = it;
                        size++;
                    }
                }
                for(int itC=0; itC < CountON-CHcapacity[CH]; itC++){
                    std::map<int, Matching::iterator>::reverse_iterator last= Order.rbegin();
                    Matched.erase(last->second);
                    bool found=false;
                    for(size_t itpB=0; itpB<ONunmatched.size(); itpB++){
                        if(ONunmatched[itpB]==last->second->second){
                            found=true;
                            break;
                        }
                    }
                    if(!found){
                        ONunmatched.push_back(last->second->second);
                    }
                    Order.erase(last->first);
                }
            }
            for(Matching::iterator it=ret.first; it!=ret.second; ++it){
                for(size_t itUn = 0; itUn<ONunmatched.size(); itUn++){
                    if(ONunmatched[itUn]==it->second){
                        ONunmatched.erase(ONunmatched.begin()+itUn);
                    }
                }
            }
        }

    }

    //raccoglie statistiche
    orgStatistic();

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
    statistics.meanCluster = statistics.numON / (statistics.numCH!=0) ? statistics.numCH : 1;

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
            if(it==PrefCHLists.end()){
                double sqrDistance = nodesCoord[CH].sqrdist(nodesCoord[ON]);
                sumexpU += calcUtility(sqrDistance);
                countON++;
            }
        }
        statistics.CHutility += (numONCluster!= 0) ? sumU/(10*numONCluster) : 0;
        statistics.expCHutility += (countON!= 0) ? sumexpU/(10*countON) : 0;
        statistics.diffCHutility += ((countON!= 0) ? sumexpU/(10*countON) : 0)-((numONCluster!= 0) ? sumU/(10*numONCluster) : 0);
    }
    statistics.meanCHutility = statistics.CHutility/(statistics.numCH!=0) ? statistics.numCH : 1;
    statistics.meanexpCHutility = statistics.expCHutility/(statistics.numCH!=0) ? statistics.numCH : 1;
    statistics.meandiffCHutility = statistics.diffCHutility/(statistics.numCH!=0) ? statistics.numCH : 1;

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
        statistics.sigmaCHutility += (((numONCluster!= 0) ? sumU/(10*numONCluster) : 0)-statistics.meanCHutility)*(((numONCluster!= 0) ? sumU/(10*numONCluster) : 0)-statistics.meanCHutility);
        statistics.sigmaexpCHutility += (((countON!= 0) ? sumexpU/(10*countON) : 0)-statistics.meanexpCHutility)*(((countON!= 0) ? sumexpU/(10*countON) : 0)-statistics.meanexpCHutility);
        statistics.sigmadiffCHutility += ((((countON!= 0) ? sumexpU/(10*countON) : 0)-((numONCluster!= 0) ? sumU/(10*numONCluster) : 0))-statistics.meandiffCHutility)*((((countON!= 0) ? sumexpU/(10*countON) : 0)-((numONCluster!= 0) ? sumU/(10*numONCluster) : 0))-statistics.meandiffCHutility);
    }
    statistics.sigmaCHutility = sqrt(statistics.sigmaCHutility/(statistics.numCH!=0) ? statistics.numCH : 1);
    statistics.sigmaexpCHutility = sqrt(statistics.sigmaexpCHutility/(statistics.numCH!=0) ? statistics.numCH : 1);
    statistics.sigmadiffCHutility = sqrt(statistics.sigmadiffCHutility/(statistics.numCH!=0) ? statistics.numCH : 1);
    statistics.sigmaCluster = sqrt(var / (statistics.numCH!=0) ? statistics.numCH : 1);
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

Matching TraCIRVVRSU11p::launchRVVMatching(Matching pMatched) {
    Matching tempMatch = pMatched;
    Matching BP = stable(tempMatch);
    std::map<int,int> S;
    while (BP.size() > 0){
        bool addB = false;
        for( Matching::iterator it = BP.begin(); it != BP.end(); it++ ){
            if(S.find(it->first)!= S.end() && S.find(it->second)== S.end()){
                add(it->first);
                addB = true;
                break;
            }
            if(S.find(it->second)!= S.end() && S.find(it->first)== S.end()){
                add(it->second);
                addB = true;
                break;
            }
        }
        if(!addB){
            satisfy(BP.begin()->first,BP.begin()->second, S, tempMatch); //soddisfo sempre la prima???
        }
    }
    return tempMatch;
}

Matching TraCIRVVRSU11p::stable(Matching m){
    Matching BP;
    for(auto const& p : PrefCHLists){
        const int CH = p.first;
        const PrefList CHpref = p.second;
        std::pair <Matching::iterator, Matching::iterator> ret;
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
                if(found==false){
                    bool insertBP = true;
                    for( Matching::iterator it = m.begin(); it != m.end(); it++ ){
                        if(it->second == ON){
                            const int onCH = it->first;
                            PrefList ONpref = PrefONLists.find(ON)->second;
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
                        //devo uscire appena trovo una blocking pair del CH???
                    }
                }
                j++;
            }
        }
    }
    return BP;
}

void TraCIRVVRSU11p::add(int a){

}

void TraCIRVVRSU11p::satisfy(int CH,int ON, std::map<int,int> &S, Matching &m){
    S[ON]=1;
    S[CH]=1;
    for( Matching::iterator it = m.begin(); it != m.end(); it++ ){
       if(it->second == ON){
           m.erase(it);
           break;
       }
    }
    if(m.count(CH)>=CHcapacity[CH]){
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
                        iWorst=jt;
                        worst = it;
                        break;
                    }
                }
            }
            if(iWorst==-1){
                worst = it;
                break;
            }
        }
        m.erase(iWorst);
    }

}
