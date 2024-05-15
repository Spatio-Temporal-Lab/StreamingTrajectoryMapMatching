/* 
 * Copyright (C) 2022  ST-Lab
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package org.urbcomp.cupid.db.model.sample;

import com.alibaba.fastjson.JSON;
import com.alibaba.fastjson.JSONArray;
import com.alibaba.fastjson.JSONObject;
import org.geotools.geojson.geom.GeometryJSON;
import org.locationtech.jts.geom.Geometry;
import org.urbcomp.cupid.db.model.Attribute;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.point.SpatialPoint;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.roadnetwork.RoadSegment;
import org.urbcomp.cupid.db.model.roadnetwork.RoadSegmentDirection;
import org.urbcomp.cupid.db.model.roadnetwork.RoadSegmentLevel;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.CoordTransformUtils;
import org.urbcomp.cupid.db.util.DataTypeUtils;
import org.urbcomp.cupid.db.util.WKTUtils;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.sql.Timestamp;
import java.util.*;
import java.util.stream.Collectors;

public class ModelGenerator {
    public static Trajectory generateTrajectory() {
        return generateTrajectory("data/output.txt");
    }

    public static Trajectory generateTrajectory(String trajFile) {
        return generateTrajectory(trajFile, -1);
    }

    public static Trajectory generateTrajectory(String trajFile, int maxLength) {
        try (
            InputStream in = ModelGenerator.class.getClassLoader().getResourceAsStream(trajFile);
            BufferedReader br = new BufferedReader(
                new InputStreamReader(Objects.requireNonNull(in))
            )
        ) {
            String trajStr = br.readLine();
            trajStr = br.readLine();
            String correctStr = trajStr.replaceFirst("\\[", "[\"").replaceFirst(",", "\",");
            List<String> result = JSON.parseArray(correctStr, String.class);
            String oid = result.get(0);
            List<String> pointsStrList = JSON.parseArray(result.get(1), String.class);
            List<GPSPoint> pointsList = pointsStrList.stream()
                    .map(o -> JSON.parseArray(o, String.class))
                    .map(o -> {
                        Timestamp timestamp = Timestamp.valueOf(o.get(0));
                        double lng = Double.parseDouble(o.get(1));
                        double lat = Double.parseDouble(o.get(2));
                        double[] convertedCoords = CoordTransformUtils.gcj02Towgs84(lng, lat);
                        return new GPSPoint(timestamp, convertedCoords[0], convertedCoords[1]);
                    })
                    .collect(Collectors.toList());
            if (maxLength > 0) {
                pointsList = pointsList.subList(0, maxLength);
            }
            return new Trajectory(oid + pointsList.get(0).getTime(), oid, pointsList);
        } catch (IOException e) {
            throw new RuntimeException("Generate trajectory error: " + e.getMessage());
        }
    }

    public static Trajectory generateTrajectory(List<String> names, List<String> types) {
        try (
            InputStream in = ModelGenerator.class.getClassLoader()
                .getResourceAsStream("data/traj1.txt");
            BufferedReader br = new BufferedReader(
                new InputStreamReader(Objects.requireNonNull(in))
            )
        ) {
            String trajStr = br.readLine();
            String correctStr = trajStr.replaceFirst("\\[", "[\"").replaceFirst(",", "\",");
            List<String> result = JSON.parseArray(correctStr, String.class);
            String oid = result.get(0);
            List<String> pointsStrList = JSON.parseArray(result.get(1), String.class);
            List<GPSPoint> pointsList = pointsStrList.stream()
                .map(o -> JSON.parseArray(o, String.class))
                .map(
                    o -> new GPSPoint(
                        Timestamp.valueOf(o.get(0)),
                        Double.parseDouble(o.get(1)),
                        Double.parseDouble(o.get(2))
                    )
                )
                .collect(Collectors.toList());
            int n = names.size();
            GeometryJSON geometryJSON = new GeometryJSON();
            Map<String, Attribute> attributeMap = new HashMap<>();
            for (int i = 0; i < n; ++i) {
                String name = names.get(i);
                String typeName = types.get(i);
                Class type = DataTypeUtils.getClass(typeName);
                attributeMap.put(
                    name,
                    new Attribute(
                        typeName,
                        Geometry.class.isAssignableFrom(type)
                            ? type.cast(geometryJSON.read(result.get(i + 2)))
                            : JSONObject.parseObject(result.get(i + 2), type)
                    )
                );
            }
            return new Trajectory(oid + pointsList.get(0).getTime(), oid, pointsList, attributeMap);
        } catch (IOException e) {
            throw new RuntimeException("Generate trajectory error: " + e.getMessage());
        }
    }

    public static Trajectory generateTrajectoryByStr(String trajStr, int maxLength) {
        String correctStr = trajStr.replaceFirst("\\[", "[\"").replaceFirst(",", "\",");
        // 解析 JSON 字符串
        JSONArray jsonArray = JSON.parseArray(correctStr);
        String oid = jsonArray.getString(0);
        JSONArray pointsArray = jsonArray.getJSONArray(1);

        // 创建 GPSPoint 对象列表
        List<GPSPoint> pointsList = new ArrayList<>();
        for (Object obj : pointsArray) {
            JSONArray point = (JSONArray) obj;
            Timestamp timestamp = Timestamp.valueOf(point.getString(0));
            double lng = point.getDouble(1);
            double lat = point.getDouble(2);
            double[] convertedCoords = CoordTransformUtils.gcj02Towgs84(lng, lat);
            pointsList.add(new GPSPoint(timestamp, convertedCoords[0], convertedCoords[1]));
        }

        // 截取指定长度的轨迹
        if (maxLength > 0 && pointsList.size() > maxLength) {
            pointsList = pointsList.subList(0, maxLength);
        }

        // 创建 Trajectory 对象并返回
        return new Trajectory(oid + pointsList.get(0).getTime(), oid, pointsList);
    }

    public static RoadSegment generateRoadSegment() {
        List<SpatialPoint> points = new ArrayList<>();
        points.add(new SpatialPoint(111.37939453125, 54.00776876193478));
        points.add(new SpatialPoint(116.3671875, 53.05442186546102));
        return new RoadSegment(1, 1, 2, points).setDirection(RoadSegmentDirection.DUAL)
            .setSpeedLimit(30.0)
            .setLevel(RoadSegmentLevel.URBAN_ROAD)
            .setLengthInMeter(120);
    }

    public static List<RoadSegment> generateRoadSegments() {
        return generateRoadSegments(-1);
    }

    public static List<RoadSegment> generateRoadSegments(int maxLength) {
        try (
            InputStream in = ModelGenerator.class.getClassLoader()
                .getResourceAsStream("data/outputchengdu.csv");
            BufferedReader br = new BufferedReader(
                new InputStreamReader(Objects.requireNonNull(in))
            )
        ) {
            br.readLine(); // read head
            String roadSegmentStr;
            List<RoadSegment> roadSegments = new ArrayList<>();
            while ((roadSegmentStr = br.readLine()) != null) {
                String[] roadStrArr = roadSegmentStr.split("\\|");
                int roadSegmentId = Integer.parseInt(roadStrArr[0]);
                int startId = Integer.parseInt(roadStrArr[2]);
                int endId = Integer.parseInt(roadStrArr[3]);
                RoadSegmentDirection direction = RoadSegmentDirection.valueOf(
                    Integer.parseInt(roadStrArr[4])
                );
                RoadSegmentLevel level = RoadSegmentLevel.valueOf(Integer.parseInt(roadStrArr[5]));
                double speedLimit = Double.parseDouble(roadStrArr[6]);
                double lengthInM = Double.parseDouble(roadStrArr[7]);
                List<SpatialPoint> points = Arrays.stream(
                    WKTUtils.read(roadStrArr[1]).getCoordinates()
                ).map(SpatialPoint::new).collect(Collectors.toList());
                RoadSegment rs = new RoadSegment(roadSegmentId, startId, endId, points);
                rs.setDirection(direction)
                    .setLevel(level)
                    .setSpeedLimit(speedLimit)
                    .setLengthInMeter(lengthInM);
                roadSegments.add(rs);
                if (maxLength >= 0 && roadSegments.size() >= maxLength) break;
            }
            return roadSegments;
        } catch (Exception e) {
            throw new RuntimeException("Generate road network error: " + e.getMessage());
        }

    }

    public static RoadNetwork generateRoadNetwork() {
        return new RoadNetwork(generateRoadSegments());
    }

    public static RoadNetwork generateRoadNetwork(int maxLength) {
        return new RoadNetwork(generateRoadSegments(maxLength));
    }
}
