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
package org.urbcomp.cupid.db.model.roadnetwork;

public enum RoadSegmentLevel {

    UNDEFINED(-1),

    ELEVATED_ROAD(0),

    HIGH_WAY_ROAD(1),

    NATIONAL_ROAD(2),

    PROVINCIAL_ROAD(3),

    COUNTRY_ROAD(4),

    MAIN_ROAD(5),

    URBAN_ROAD(6),

    DOWNTOWN_ROAD(7),

    RESIDENTIAL_ROAD(8),

    SIDEWALK_ROAD(9);

    private final int value;

    RoadSegmentLevel(int value) {
        this.value = value;
    }

    public static RoadSegmentLevel valueOf(int value) {
        for (RoadSegmentLevel type : values()) {
            if (type.value() == value) {
                return type;
            }
        }
        return null;
    }

    public int value() {
        return this.value;
    }
}
