package org.urbcomp.cupid.db.weight;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.geojson.Feature;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.FeatureCollectionWithProperties;

import java.io.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;


public class loadData {

    private final List<List<Feature>> featureLists = new ArrayList<>();

    private final List<Trajectory> trajectoryList = new ArrayList<>();

    public loadData(int loadNum) {
        load(loadNum);
    }

    private void load(int loadNum) {
        String loadLabelFile = "D:\\StreamingTrajectoryMapMatching\\src\\main\\resources\\data\\map_matched_trajectories_labels.geojson";
        String loadTraFile = "D:\\StreamingTrajectoryMapMatching\\src\\main\\resources\\data\\output.txt";
        try (BufferedReader br1 = new BufferedReader(new FileReader(loadLabelFile));
             BufferedReader br2 = new BufferedReader(new FileReader(loadTraFile))
        ) {
            String line;
            String line2;
            int index = 0;
            while ((line = br1.readLine()) != null && (line2 = br2.readLine()) != null && (index < loadNum || loadNum == -1)) {
                index++;
                FeatureCollectionWithProperties fcp = new ObjectMapper().readValue(line, FeatureCollectionWithProperties.class);
                List<Feature> features = fcp.getFeatures();
                featureLists.add(features);
                trajectoryList.add(ModelGenerator.generateTrajectoryByStr(line2, 0));
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public List<List<Feature>> getFeatureList() {
        return this.featureLists;
    }

    public List<Trajectory> getTrajectoryList() {
        return this.trajectoryList;
    }


}
