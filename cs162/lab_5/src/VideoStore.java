/*! \file VideoStore.java
 *  \author Soo-Hyun Yoo
 */

public class VideoStore {
    private ArrayList<Video> inventory;
    private ArrayList<RentalCard> cards;

    // Rent a video from the store.
    public int rentVideo(String vidTitle, Customer customer) {
        for (Video vid : this.inventory) {    // Cycle through videos in inventory.
            if (vid.getTitle == vidTitle) {   // Check if titles match.
                customer.card.addVideo(vid);  // Add video to card.
                this.inventory.remove(vid);   // Remove video from inventory.
                this.cards.add(card);         // Add card to cards list.
                return 0;
            }
        }
        return 1;
    }

    // Return video to the store.
    public int returnVideo(String vidTitle, Customer customer) {
        for (RentalCard card : this.cards) {       // Cycle through the cards.
            for (Video vid : card.getVideos()) {   // Cycle through the videos in each card.
                if (vid.getTitle == vidTitle) {    // Check if titles match.
                    card.removeVideo(vid);         // Remove video from card.
                    customer.card = card;
                    this.inventory.add(vid);       // Add video to inventory.
                    if (card.getVideos().length == 0) {   // If card has no videos...
                        this.cards.remove(card);          // ...remove card from card list.
                    }
                    return 0;
                }
            }
        }
        return 1;
    }

    // Recommend a video.
    public ArrayList<Video> recommendVideo(Customer customer) {
        // Cycle through the videos in the cards list and add the video to a recommendation list if the card is that of a customer of the same gender and similar age as the customer calling recommendVideo. Return the list of recommended videos.
    }


    public void VideoStore() {
        this.inventory = new ArrayList<Video>;
        this.cards = new ArrayList<RentalCard>;
    }

}
