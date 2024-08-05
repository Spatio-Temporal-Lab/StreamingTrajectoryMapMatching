package org.urbcomp.cupid.db.weight;

import com.microsoft.ml.lightgbm.*;
import org.apache.commons.lang3.StringUtils;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TimeStep;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.util.GeoFunctions;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;

public class LightGBMModelLoad {

    private SWIGTYPE_p_void boosterPtr;
    private String modelString;

    public LightGBMModelLoad(String modelString) {
        this.modelString = modelString;
        initModel();
    }

    public void initModel() {
        try {
            init(modelString);
        } catch (Exception e) {
            throw new RuntimeException("模型加载失败", e);
        }
    }

    public void init(String modelString) throws Exception {
        initEnv();
        if (StringUtils.isEmpty(modelString)) {
            throw new Exception("the input model string must not be null");
        }
        this.boosterPtr = getBoosterPtrFromModelString(modelString);
    }

    private void initEnv() throws IOException {
        String osPrefix = NativeLoader.getOSPrefix();
        new NativeLoader("/com/microsoft/ml/lightgbm").loadLibraryByName(osPrefix + "_lightgbm");
        new NativeLoader("/com/microsoft/ml/lightgbm").loadLibraryByName(osPrefix + "_lightgbm_swig");
    }

    private String getOSPrefix() {
        String os = System.getProperty("os.name").toLowerCase();
        if (os.contains("win")) {
            return "windows";
        } else if (os.contains("mac")) {
            return "osx";
        } else {
            return "lib";
        }
    }

    private void validate(int result) throws Exception {
        if (result == -1) {
            throw new Exception("Booster LoadFromString call failed in LightGBM with error: " + lightgbmlib.LGBM_GetLastError());
        }
    }

    private SWIGTYPE_p_void getBoosterPtrFromModelString(String lgbModelString) throws Exception {
        SWIGTYPE_p_p_void boosterOutPtr = lightgbmlib.voidpp_handle();
        SWIGTYPE_p_int numItersOut = lightgbmlib.new_intp();
        validate(
                lightgbmlib.LGBM_BoosterLoadModelFromString(lgbModelString, numItersOut, boosterOutPtr)
        );
        return lightgbmlib.voidpp_value(boosterOutPtr);
    }

    /**
     * 预测
     * @param data 批量向量
     * @param numRows 预测行数
     * @param numFeatures 向量大小
     * @return 批量预测结果
     */
    public double[] predictForMat(double[] data, int numRows, int numFeatures) {
        int data64bitType = lightgbmlibConstants.C_API_DTYPE_FLOAT64;
        int isRowMajor = 1;
        String datasetParams = "";
        SWIGTYPE_p_double scoredDataOutPtr = lightgbmlib.new_doubleArray(numRows * numFeatures);

        SWIGTYPE_p_long_long scoredDataLengthLongPtr = lightgbmlib.new_int64_tp();
        lightgbmlib.int64_tp_assign(scoredDataLengthLongPtr, numRows * numFeatures);

        SWIGTYPE_p_double doubleArray = lightgbmlib.new_doubleArray(data.length);
        for (int i = 0; i < data.length; i++) {
            lightgbmlib.doubleArray_setitem(doubleArray, i, data[i]);
        }
        SWIGTYPE_p_void pdata = lightgbmlib.double_to_voidp_ptr(doubleArray);

        try {
            lightgbmlib.LGBM_BoosterPredictForMat(
                    boosterPtr,
                    pdata,
                    data64bitType,
                    numRows,
                    numFeatures,
                    isRowMajor,
                    0,
                    -1,
                    datasetParams,
                    scoredDataLengthLongPtr,
                    scoredDataOutPtr);
            return predToArray(scoredDataOutPtr, numRows);
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println(lightgbmlib.LGBM_GetLastError());
        } finally {
            lightgbmlib.delete_doublep(doubleArray);
            lightgbmlib.delete_doublep(scoredDataOutPtr);
            lightgbmlib.delete_int64_tp(scoredDataLengthLongPtr);
        }
        return new double[numRows];
    }

    private double[] predToArray(SWIGTYPE_p_double scoredDataOutPtr, int numRows) {
        double[] res = new double[numRows];
        for (int i = 0; i < numRows; i++) {
            res[i] = lightgbmlib.doubleArray_getitem(scoredDataOutPtr, i);
        }
        return res;
    }

    /**
     * 根据给定的特征数据进行预测
     * @param currTimeStep 当前时间步的候选点
     * @param prePoint 上一个点
     * @param currPoint 当前点
     * @return 预测结果
     */
    public double predict(TimeStep currTimeStep, GPSPoint prePoint, GPSPoint currPoint) {
        double sum = 0;
        List<Double> allDistance = new ArrayList<>();
        for (CandidatePoint p : currTimeStep.getCandidates()) {
            allDistance.add(p.getOffsetInMeter());
            sum += p.getOffsetInMeter();
        }

        Map<String, Double> featuresMap = new HashMap<>();
        featuresMap.put("avgDistance", sum / allDistance.size());
        featuresMap.put("roadNum", (double) currTimeStep.getCandidates().size());
        featuresMap.put("distanceBtwPreAndCurr", GeoFunctions.getDistanceInM(prePoint.getLng(), prePoint.getLat(), currPoint.getLng(), currPoint.getLat()));
        featuresMap.put("timeBtwPreAndCurr", (double) ((currPoint.getTime().getTime() - prePoint.getTime().getTime()) / 1000));
        featuresMap.put("prePointLng", prePoint.getLng());
        featuresMap.put("prePointLat", prePoint.getLat());
        featuresMap.put("currPointLat", currPoint.getLat());
        featuresMap.put("currPointLng", currPoint.getLng());

        // 将特征转换为数组
        double[] features = featuresMap.values().stream().mapToDouble(Double::doubleValue).toArray();

        // 调用预测方法
        double[] result = predictForMat(features, 1, features.length);

        return result[0];
    }

}
