import org.dmg.pmml.FieldName;
import org.dmg.pmml.PMML;
import org.jpmml.evaluator.*;
import org.xml.sax.SAXException;


import javax.xml.bind.JAXBException;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class testACC {
    public static void main(String[] args) {
        Evaluator evaluator;
        try {
            String modelPath = "src/main/java/org/urbcomp/cupid/db/model/decision_tree_model.pmml";
            PMML pmml = org.jpmml.model.PMMLUtil.unmarshal(Files.newInputStream(new File(modelPath).toPath()));
            ModelEvaluatorFactory modelEvaluatorFactory = ModelEvaluatorFactory.newInstance();
            evaluator = modelEvaluatorFactory.newModelEvaluator(pmml);
        } catch (IOException | JAXBException | SAXException e) {
            throw new RuntimeException("Error loading PMML model", e);
        }

        Map<String, Integer> featuresMap = new HashMap<>();
        featuresMap.put("currRoadSegmentId", 61271);
        featuresMap.put("nextRoadSegmentId", 79146);
        featuresMap.put("hour", 16);
        featuresMap.put("dayofweek", 7);

        Map<FieldName, FieldValue> arguments = new HashMap<>();
        for (InputField inputField : evaluator.getInputFields()) {
            System.out.println(inputField);
            FieldName inputFieldName = inputField.getName();
            Object rawValue = featuresMap.get(inputFieldName.getValue());
            FieldValue inputFieldValue = inputField.prepare(rawValue);
            arguments.put(inputFieldName, inputFieldValue);
        }

        Map<FieldName, ?> results = evaluator.evaluate(arguments);
        List<TargetField> targetFields = evaluator.getTargetFields();
        System.out.println(targetFields);

        TargetField targetField = targetFields.get(0);
        FieldName targetFieldName = targetField.getName();

        Object targetFieldValue = results.get(targetFieldName);

        double primitiveValue = -1;
        if (targetFieldValue instanceof Computable) {
            Computable computable = (Computable) targetFieldValue;
            primitiveValue = (double) computable.getResult();
        }
        System.out.println(primitiveValue);

    }


}
