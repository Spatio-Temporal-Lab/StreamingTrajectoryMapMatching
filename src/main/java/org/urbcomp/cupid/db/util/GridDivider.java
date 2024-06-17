package org.urbcomp.cupid.db.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class GridDivider {

    // 地球半径（单位：米）
    private static final double EARTH_RADIUS = 6378137;

    // 度转换为弧度
    private static double toRadians(double degree) {
        return degree * Math.PI / 180.0;
    }

    // 弧度转换为度
    private static double toDegrees(double radian) {
        return radian * 180.0 / Math.PI;
    }

    // 根据距离（米）计算纬度变化量
    private static double latitudeDelta(double distance) {
        return toDegrees(distance / EARTH_RADIUS);
    }

    // 根据距离（米）和纬度计算经度变化量
    private static double longitudeDelta(double distance, double latitude) {
        double radiusAtLatitude = EARTH_RADIUS * Math.cos(toRadians(latitude));
        return toDegrees(distance / radiusAtLatitude);
    }

    // 划分网格并返回经纬度网格列表
    public static List<double[]> divideGrid(double minLat, double maxLat, double minLon, double maxLon, double gridSize) {
        List<double[]> grids = new ArrayList<>();

        double latStep = latitudeDelta(gridSize);
        double lonStep = longitudeDelta(gridSize, (minLat + maxLat) / 2);

        for (double lat = minLat; lat <= maxLat; lat += latStep) {
            for (double lon = minLon; lon <= maxLon; lon += lonStep) {
                grids.add(new double[]{lat, lon});
            }
        }

        return grids;
    }

    // 根据给定经纬度返回网格ID（x, y）
    public static int point2grid(double lon, double lat, List<Double> lonVec, List<Double> latVec) {
        int lonI = 0;
        int latI = 0;

        // 计算经度索引
        for (int i = 0; i < lonVec.size(); i++) {
            if (lonVec.get(i) < lon) {
                lonI++;
            } else {
                break;
            }
        }

        // 计算纬度索引
        for (int i = 0; i < latVec.size(); i++) {
            if (latVec.get(i) < lat) {
                latI++;
            } else {
                break;
            }
        }

        // 检查索引是否超出边界
        if (lonI == 0 || latI == 0 || lonI == lonVec.size() || latI == latVec.size()) {
            return -1;
        } else {
            return (latI - 1) * (lonVec.size() - 1) + lonI;
        }
    }

    public static void main(String[] args) {
        double minLat = 34.0;
        double maxLat = 35.0;
        double minLon = -118.0;
        double maxLon = -117.0;
        double gridSize = 1000; // 网格大小（单位：米）

        // 划分网格并生成经纬度列表
        List<double[]> grids = divideGrid(minLat, maxLat, minLon, maxLon, gridSize);

        // 提取经纬度向量
        List<Double> lonVec = new ArrayList<>();
        List<Double> latVec = new ArrayList<>();
        for (double[] grid : grids) {
            if (!lonVec.contains(grid[1])) {
                lonVec.add(grid[1]);
            }
            if (!latVec.contains(grid[0])) {
                latVec.add(grid[0]);
            }
        }

        // 测试点
        double lon = -117.75;
        double lat = 34.25;

        // 根据经纬度返回网格ID
        int gridId = point2grid(lon, lat, lonVec, latVec);

        System.out.println("Grid ID: " + gridId);
    }
}
