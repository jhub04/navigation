public class Preprocessing {
  static int R = 6371;


  double haversine(double lat1, double lat2, double long1, double long2) {
    double avg_latitude = (lat1 - lat2) / 2;
    double avg_longitude = (long1 - long2) / 2;

    double sqrt = Math.sqrt(Math.sin(toRadians(avg_latitude)) * Math.sin(toRadians(avg_latitude)) + Math.cos(toRadians(lat1)) * Math.cos(toRadians(lat2)) * Math.sin(toRadians(avg_longitude)) * Math.sin(toRadians(avg_longitude)));
    return 2*R * Math.asin(sqrt);

  }

  double toRadians (double degrees) {
    return degrees * (Math.PI / 180);
  }

  public static void main(String[] args) {
    double lat1 = 65.6107182;
    double long1 = -16.9173984;
    double lat2 = 64.1184002;
    double long2 = -21.8015604;

    Preprocessing pp = new Preprocessing();
    double result = pp.haversine(lat1, lat2, long1, long2);

    System.out.println(result);
  }

}
