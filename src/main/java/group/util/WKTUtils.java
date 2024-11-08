package group.util;

import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.io.ParseException;
import org.locationtech.jts.io.WKTReader;
import org.locationtech.jts.io.WKTWriter;

public class WKTUtils {
    private static final ThreadLocal<WKTReader> readerPool = ThreadLocal.withInitial(
        WKTReader::new
    );
    private static final ThreadLocal<WKTWriter> writerPool = ThreadLocal.withInitial(
        WKTWriter::new
    );

    public static Geometry read(String s) throws ParseException {
        return readerPool.get().read(s);
    }

    public static String write(Geometry g) {
        return writerPool.get().write(g);
    }
}
