import org.geojson.Feature;
import org.junit.Before;
import org.junit.Test;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.weight.findWeight;
import org.urbcomp.cupid.db.weight.loadData;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class generateData {

    private List<List<Feature>> featureList;
    private List<Trajectory> trajectories;
    private int generateNum;
    private findWeight fdW;


    @Before
    public void setUpBeforeClass() {
        loadData data = new loadData(-1);
        featureList = data.getFeatureList();
        trajectories = data.getTrajectoryList();
        generateNum = Math.min(100, featureList.size());
        fdW = new findWeight();
    }

    @Test
    public void generateWeight() {
        String writeFileName = "weight.txt";
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(writeFileName))) {
            String header = "weight,avgDistance,roadNum,disBtwPreAndCurr,timeBtwPreAndCurr,bearing";
            writer.write(header);
            writer.newLine();
            for (int i = 0; i < generateNum; i++) {
                List<Double> temp = new ArrayList<>();
                for (int j = 0; j < featureList.get(i).size() - 1; j++) {
                    List<Double> weightForATra = new ArrayList<>();
                    fdW.getWeight(featureList.get(i).get(j), featureList.get(i).get(j + 1),
                            trajectories.get(i).getGPSPointList().get(j), trajectories.get(i).getGPSPointList().get(j + 1), weightForATra);
                    // 写文件
                    writeWeightsToFile(writer, weightForATra);
//                    temp.add(weightForATra.get(0));
                }
//                writeWeightsToFile(writer, temp);
                System.out.println("index : " + (i + 1) + " complete!");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Test
    public void generateAvgDistance() {
        String writeFileName = "avgDistance.txt";
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(writeFileName))) {
            for (int i = 0; i < generateNum; i++) {
                List<Double> avgDistanceForATra = new ArrayList<>();
                for (int j = 0; j < featureList.get(i).size() - 1; j++) {
                    double avgDistance = fdW.getAvgDistance(trajectories.get(i).getGPSPointList().get(j + 1));
                    avgDistanceForATra.add(avgDistance);
                }
                // 写文件
                writeWeightsToFile(writer, avgDistanceForATra);
                System.out.println("index : " + (i + 1) + " complete!");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Test
    public void generateRoadNum() {
        String writeFileName = "roadNum.txt";
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(writeFileName))) {
            for (int i = 0; i < generateNum; i++) {
                List<Double> roadNumForATra = new ArrayList<>();
                for (int j = 0; j < featureList.get(i).size() - 1; j++) {
                    double roadNum = fdW.getRoadNum(trajectories.get(i).getGPSPointList().get(j + 1));
                    roadNumForATra.add(roadNum);
                }
                // 写文件
                writeWeightsToFile(writer, roadNumForATra);
                System.out.println("index : " + (i + 1) + " complete!");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Test
    public void generateDisBtwPreAndCurr() {
        String writeFileName = "distanceBtwPreAndCurr.txt";
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(writeFileName))) {
            for (int i = 0; i < generateNum; i++) {
                List<Double> avgDistanceForATra = new ArrayList<>();
                for (int j = 0; j < featureList.get(i).size() - 1; j++) {
                    double avgDistance = fdW.getDisBtwPreAndCurr(trajectories.get(i).getGPSPointList().get(j), trajectories.get(i).getGPSPointList().get(j + 1));
                    avgDistanceForATra.add(avgDistance);
                }
                // 写文件
                writeWeightsToFile(writer, avgDistanceForATra);
                System.out.println("index : " + (i + 1) + " complete!");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Test
    public void generateTimeBtwPreAndCurr() {
        String writeFileName = "timeBtwPreAndCurr.txt";
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(writeFileName))) {
            for (int i = 0; i < generateNum; i++) {
                List<Double> avgTimeForATra = new ArrayList<>();
                for (int j = 0; j < featureList.get(i).size() - 1; j++) {
                    double avgTime = fdW.getTimeBtwPreAndCurr(trajectories.get(i).getGPSPointList().get(j), trajectories.get(i).getGPSPointList().get(j + 1));
                    avgTimeForATra.add(avgTime);
                }
                // 写文件
                writeWeightsToFile(writer, avgTimeForATra);
                System.out.println("index : " + (i + 1) + " complete!");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Test
    public void generatePrePointLng() {
        String writeFileName = "prePointLng.txt";
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(writeFileName))) {
            for (int i = 0; i < generateNum; i++) {
                List<Double> prePointLngForATra = new ArrayList<>();
                for (int j = 0; j < featureList.get(i).size() - 1; j++) {
                    prePointLngForATra.add(trajectories.get(i).getGPSPointList().get(j).getLng());
                }
                // 写文件
                writeWeightsToFile(writer, prePointLngForATra);
                System.out.println("index : " + (i + 1) + " complete!");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        System.out.println(trajectories.get(0).getGPSPointList());
    }

    @Test
    public void generatePrePointLat() {
        String writeFileName = "prePointLat.txt";
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(writeFileName))) {
            for (int i = 0; i < generateNum; i++) {
                List<Double> prePointLatForATra = new ArrayList<>();
                for (int j = 0; j < featureList.get(i).size() - 1; j++) {
                    prePointLatForATra.add(trajectories.get(i).getGPSPointList().get(j).getLat());
                }
                // 写文件
                writeWeightsToFile(writer, prePointLatForATra);
                System.out.println("index : " + (i + 1) + " complete!");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Test
    public void generateCurrPointLng() {
        String writeFileName = "currPointLng.txt";
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(writeFileName))) {
            for (int i = 0; i < generateNum; i++) {
                List<Double> currPointLngForATra = new ArrayList<>();
                for (int j = 0; j < featureList.get(i).size() - 1; j++) {
                    currPointLngForATra.add(trajectories.get(i).getGPSPointList().get(j + 1).getLng());
                }
                // 写文件
                writeWeightsToFile(writer, currPointLngForATra);
                System.out.println("index : " + (i + 1) + " complete!");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Test
    public void generateCurrPointLat() {
        String writeFileName = "currPointLat.txt";
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(writeFileName))) {
            for (int i = 0; i < generateNum; i++) {
                List<Double> currPointLatForATra = new ArrayList<>();
                for (int j = 0; j < featureList.get(i).size() - 1; j++) {
                    currPointLatForATra.add(trajectories.get(i).getGPSPointList().get(j + 1).getLat());
                }
                // 写文件
                writeWeightsToFile(writer, currPointLatForATra);
                System.out.println("index : " + (i + 1) + " complete!");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Test
    public void generateTrue() {
        String writeFileName1 = "all.txt";
        try (BufferedWriter writer1 = new BufferedWriter(new FileWriter(writeFileName1))) {
            String header = "weight,avgDisBtwPreAndCurr,disBtwPreAndCurr,timeBtwPreAndCurr,roadNum,avgDistanceToRoad,distanceToRoad,bearing";
            writer1.write(header);
            writer1.newLine();
            for (int i = 0; i < generateNum; i++) {
                for (int j = 0; j < featureList.get(i).size() - 1; j++) {
                    List<List<Double>> allForAPoint = new ArrayList<>();
                    fdW.calAllFeatureForAPoint(allForAPoint, featureList.get(i).get(j), featureList.get(i).get(j + 1),
                            trajectories.get(i).getGPSPointList().get(j), trajectories.get(i).getGPSPointList().get(j + 1));
                    for (List<Double> doubles : allForAPoint) {
                        writeWeightsToFile(writer1, doubles);
                    }
                }
                System.out.println("index : " + (i + 1) + " complete!");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static void writeWeightsToFile(BufferedWriter writer, List<Double> weights) throws IOException {
        String line = String.join(",", weights.stream().map(Object::toString).toArray(String[]::new));
        writer.write(line);
        writer.newLine();
    }
}
