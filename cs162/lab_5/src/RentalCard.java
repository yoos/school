/*! \file RentalCard.java
 *  \author Soo-Hyun Yoo
 */

public class RentalCard {
    private String customerName;
    private ArrayList<Video> rentedVideos;

    public ArrayList<Video> getVideos() {
        return this.rentedVideos;
    }

    public int addVideo(Video vid) {
        this.rentedVideos.add(vid);
        return 0;
    }

    public int removeVideo(Video vid) {
        this.rentedVideos.remove(vid);
        return 0;
    }

    public void RentalCard(String customerName) {
        this.customerName = customerName;
    }

}
