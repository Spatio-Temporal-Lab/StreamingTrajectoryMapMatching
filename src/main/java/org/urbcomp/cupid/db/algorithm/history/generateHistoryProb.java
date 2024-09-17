package org.urbcomp.cupid.db.algorithm.history;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.geojson.Feature;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TimeStep;
import org.urbcomp.cupid.db.model.roadnetwork.RoadGraph;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.roadnetwork.RoadSegment;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.util.FeatureCollectionWithProperties;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.sql.Timestamp;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

public class generateHistoryProb {

    private HashMap<String, Integer> historyProb;
    private HashMap<String, Integer> allCount;

    private final RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();

    private final RoadGraph roadGraph = roadNetwork.getDirectedRoadGraph();

    public generateHistoryProb(){
        historyProb = new HashMap<>();
        allCount = new HashMap<>();
        loadProb(2000);
        System.out.println("finish load!");
    }

    public void loadProb(int loadNum) {
        String loadLabelFile = "D:\\StreamingTrajectoryMapMatching\\src\\main\\resources\\data\\map_matched_trajectories_labels.geojson";
        try (BufferedReader br1 = new BufferedReader(new FileReader(loadLabelFile));
        ) {
            String line;
            int index = 0;
            getAllGraphOneHopReachableEdges();
            while ((line = br1.readLine()) != null && (index < loadNum || loadNum == -1)) {
                index++;
                FeatureCollectionWithProperties fcp = new ObjectMapper().readValue(line, FeatureCollectionWithProperties.class);
                List<Feature> features = fcp.getFeatures();
                for (int i = 0; i < features.size() - 1; i++) {
                    int preRoadId = features.get(i).getProperty("roadSegmentId");
                    int currRoadId = features.get(i + 1).getProperty("roadSegmentId");
                    String timeString = features.get(i + 1).getProperty("time");
                    SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.S");
                    Date date = sdf.parse(timeString);
                    Timestamp timestamp = new Timestamp(date.getTime());
                    int week = timestamp.toLocalDateTime().getDayOfWeek().getValue();
                    if (preRoadId != currRoadId) {
                        allCount.put(preRoadId + "#" + week, allCount.get(preRoadId + "#" + week) + 1);
                        historyProb.put(preRoadId + "#" + currRoadId + "#" + week , historyProb.getOrDefault(preRoadId + "#" + currRoadId + "#" + week, 0) + 1);
                    }
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
    }

    public void getAllGraphOneHopReachableEdges(){
        Set<RoadSegment> roadSegments = roadNetwork.getDirectedRoadGraph().edgeSet();
        for (RoadSegment rs : roadSegments){
            Set<RoadSegment> oneHopReachableRoadSegments = roadGraph.getOneHopReachableEdges(roadNetwork.getRoadSegmentById(rs.getRoadSegmentId()));
            for (int i = 1; i <= 7; i++){
                allCount.put(rs.getRoadSegmentId() + "#" + i , oneHopReachableRoadSegments.size());
                for (RoadSegment oneHotReachableRs :  oneHopReachableRoadSegments){
                    historyProb.put(rs.getRoadSegmentId() + "#" + oneHotReachableRs.getRoadSegmentId() + "#" + i, historyProb.getOrDefault(rs.getRoadSegmentId() + "#" + oneHotReachableRs.getRoadSegmentId() + "#" + i, 0) + 1);
                }
            }
        }
    }


    public double getHisProb(int preRoadId, int currRoadId, int week) {
        int allCounts = allCount.getOrDefault(preRoadId + "#" + week , -1);
        int hisCounts = historyProb.getOrDefault(preRoadId + "#" + currRoadId + "#" + week, -1);
        if (allCounts == -1 || hisCounts == -1) {
            return -1;
        } else {
            return hisCounts * 1.0 / allCounts;
        }
    }

    public void getAdjust(int id){
        Set<RoadSegment> oneHopReachableRoadSegments = roadGraph.getOneHopReachableEdges(roadNetwork.getRoadSegmentById(id));
        for (RoadSegment rs : oneHopReachableRoadSegments){
            System.out.println(rs.getRoadSegmentId());
        }
    }


}
