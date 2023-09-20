# Haversine
An arduino library to calculate azimuth angle and distance between two GPS cordinates based on haversine formula.

## Methods
toRadians(double degrees) - returns (double) degree to radians.
toDegrees(double radians) - returns (double) radians to degeree.
calculateAzimuth(double lat1, double lon1, double lat2, double lon2) - returns (double) azimuth angle in degree.
calculateDistance(double lat1, double lon1, double lat2, double lon2) - returns (double) distance in meters.
