/*! \file Video.java
 *  \author Soo-Hyun Yoo
 */

public class Video {
    private String title;
    private String genre;
    private int rating;


    public String getTitle() {
        return this.title;
    }

    public String getGenre() {
        return this.genre;
    }

    public int getRating() {
        return this.rating;
    }


    public void Video(String title, String genre, int rating) {
        this.title = title;
        this.genre = genre;
        this.rating = rating;
    }

}
