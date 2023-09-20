# Haversine
An arduino library to calculate azimuth angle and distance between two GPS cordinates based on haversine formula.
<br>
## Methods
toRadians(double degrees) - returns (double) degree to radians.<br>
toDegrees(double radians) - returns (double) radians to degeree.<br>
calculateAzimuth(double lat1, double lon1, double lat2, double lon2) - returns (double) azimuth angle in degree.<br>
calculateDistance(double lat1, double lon1, double lat2, double lon2) - returns (double) distance in meters.<br>
