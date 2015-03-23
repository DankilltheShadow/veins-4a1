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

#include "veins/modules/application/traci/TraCIGSRSU11p.h"
#include <iostream>
#include <fstream>

using Veins::AnnotationManagerAccess;

Define_Module(TraCIGSRSU11p);

void TraCIGSRSU11p::Statistics::initialize()
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

void TraCIGSRSU11p::Statistics::recordScalars(cSimpleModule& module)
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

void TraCIGSRSU11p::initialize(int stage) {
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

void TraCIGSRSU11p::onBeacon(WaveShortMessage* wsm) {

}

void TraCIGSRSU11p::onData(WaveShortMessage* wsm) {
	findHost()->getDisplayString().updateWith("r=16,green");

	annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobi->getCurrentPosition(), "blue"));

	if (!sentMessage) sendMessage(wsm->getWsmData());
}

void TraCIGSRSU11p::sendMessage(std::string blockedRoadId) {
	sentMessage = true;
	t_channel channel = dataOnSch ? type_SCH : type_CCH;
	WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
	wsm->setWsmData(blockedRoadId.c_str());
	sendWSM(wsm);
}

void TraCIGSRSU11p::sendWSM(WaveShortMessage* wsm) {
	sendDelayedDown(wsm,individualOffset);
}

void TraCIGSRSU11p::handleLowerMsg(cMessage* msg) {

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

void TraCIGSRSU11p::onPreferenceList(WaveShortMessage* wsm) {
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
void TraCIGSRSU11p::handleSelfMsg(cMessage* msg) {
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

void TraCIGSRSU11p::launchMatching() {

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

double TraCIGSRSU11p::calcUtility(double sqrD){
    double sigmaQ=FWMath::dBm2mW(-110); //converto i dBm in mW per sigma quadro
    double pDivSigma = 10/sigmaQ; // 10 mW
    double w = 20; //MHZ
    double g = 1/sqrD;

    return(w*log(1+(pDivSigma*g)));
}

void TraCIGSRSU11p::orgStatistic() {

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
