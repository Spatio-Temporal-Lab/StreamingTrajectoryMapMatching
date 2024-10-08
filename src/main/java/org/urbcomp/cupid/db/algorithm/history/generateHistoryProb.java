package org.urbcomp.cupid.db.algorithm.history;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.geojson.Feature;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TimeStep;
import org.urbcomp.cupid.db.model.roadnetwork.RoadGraph;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.roadnetwork.RoadSegment;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.util.FeatureCollectionWithProperties;

import java.io.*;
import java.sql.Timestamp;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

public class generateHistoryProb {
    /**
     * 某一对路段的历史转移概率
     * key: preRoadId#currRoadId#week
     * value: probability, 历史转移概率
     */
    private final HashMap<String, Integer> historyProb;

    /**
     * 某条路段的邻接边数
     * key: id, 路段 id+1
     * value: count, 邻接边数量
     */
    private final HashMap<String, Integer> allCount;

    private final RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();

    private final RoadGraph roadGraph = roadNetwork.getDirectedRoadGraph();

    private final ObjectMapper objectMapper = new ObjectMapper();

    /**
     * historyProb所在路径
     */
    private final String historyProbFilePath = "C:\\Users\\t1anyu\\Desktop\\Projects\\StreamingTrajectoryMapMatching\\src\\main\\java\\org\\urbcomp\\cupid\\db\\algorithm\\history\\historyProb.json";

    /**
     * allCount所在路径
     */
    private final String allCountFilePath = "C:\\Users\\t1anyu\\Desktop\\Projects\\StreamingTrajectoryMapMatching\\src\\main\\java\\org\\urbcomp\\cupid\\db\\algorithm\\history\\allCount.json";

    public generateHistoryProb() {
        historyProb = new HashMap<>();
        allCount = new HashMap<>();
//        if (!new File(historyProbFilePath).exists() && new File(allCountFilePath).exists()) { loadProb(2000); }
//        else loadProbFromFile();
//        loadProb(2000);
        System.out.println("finish load!");
    }

    /**
     * 加载所有历史匹配轨迹，计算历史转移概率
     * @param loadNum 加载历史轨迹的条数
     */
    public void loadProb(int loadNum) {
        String loadLabelFile = "C:\\Users\\t1anyu\\Desktop\\Projects\\StreamingTrajectoryMapMatching\\src\\main\\resources\\data\\map_matched_trajectories_labels.geojson";
        try (BufferedReader br1 = new BufferedReader(new FileReader(loadLabelFile))) {
            String line;
            int index = 0;
            getAllGraphOneHopReachableEdges();
            while ((line = br1.readLine()) != null && (index < loadNum || loadNum == -1)) {
                index++;
                FeatureCollectionWithProperties fcp = objectMapper.readValue(line, FeatureCollectionWithProperties.class);
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
                        allCount.put(preRoadId + "#" + week, allCount.getOrDefault(preRoadId + "#" + week, 0) + 1);
                        historyProb.put(preRoadId + "#" + currRoadId + "#" + week, historyProb.getOrDefault(preRoadId + "#" + currRoadId + "#" + week, 0) + 1);
                    }
                }
            }
            saveProbToFile(); // 保存到文件
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
    }

    private void saveProbToFile() {
        try {
            objectMapper.writeValue(new FileWriter(historyProbFilePath), historyProb);
            objectMapper.writeValue(new FileWriter(allCountFilePath), allCount);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void loadProbFromFile() {
        try {
            if (new File(historyProbFilePath).exists()) {
                HashMap<String, Integer> loadedHistoryProb = objectMapper.readValue(new FileReader(historyProbFilePath), HashMap.class);
                historyProb.putAll(loadedHistoryProb);
            }
            if (new File(allCountFilePath).exists()) {
                HashMap<String, Integer> loadedAllCount = objectMapper.readValue(new FileReader(allCountFilePath), HashMap.class);
                allCount.putAll(loadedAllCount);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void getAllGraphOneHopReachableEdges() {
        Set<RoadSegment> roadSegments = roadNetwork.getDirectedRoadGraph().edgeSet();
        for (RoadSegment rs : roadSegments) {
            Set<RoadSegment> oneHopReachableRoadSegments = roadGraph.getOneHopReachableEdges(roadNetwork.getRoadSegmentById(rs.getRoadSegmentId()));
            for (int i = 1; i <= 7; i++) {
                allCount.put(rs.getRoadSegmentId() + "#" + i, oneHopReachableRoadSegments.size());
                for (RoadSegment oneHotReachableRs : oneHopReachableRoadSegments) {
                    historyProb.put(rs.getRoadSegmentId() + "#" + oneHotReachableRs.getRoadSegmentId() + "#" + i, historyProb.getOrDefault(rs.getRoadSegmentId() + "#" + oneHotReachableRs.getRoadSegmentId() + "#" + i, 0) + 1);
                }
            }
        }
    }

    /**
     * 给定当前时间和一对路段的id，计算该对路段的历史转移概率
     * @param preRoadId 上一条路段的 id
     * @param currRoadId 当前路段的 id
     * @param week 当前时间（一周的第几天）
     * @return 该对路段的历史转移概率
     */
    public double getHisProb(int preRoadId, int currRoadId, int week) {
        int allCounts = allCount.getOrDefault(preRoadId + "#" + week, -1);
        int hisCounts = historyProb.getOrDefault(preRoadId + "#" + currRoadId + "#" + week, -1);
        if (allCounts == -1 || hisCounts == -1) {
            return -1;
        } else {
            return hisCounts * 1.0 / allCounts;
        }
    }

    /**
     * 给定id，获取对应路段的所有邻接边
     * @param id 路段的id
     */
    public void getAdjust(int id) {
        Set<RoadSegment> oneHopReachableRoadSegments = roadGraph.getOneHopReachableEdges(roadNetwork.getRoadSegmentById(id));
        for (RoadSegment rs : oneHopReachableRoadSegments) {
            System.out.println(rs.getRoadSegmentId());
        }
    }

}
