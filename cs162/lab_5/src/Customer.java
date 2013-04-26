/*! \file Customer.java
 *  \author Soo-Hyun Yoo
 */

public class Customer {
    private String name;
    private int age;
    private char gender;

    public RentalCard card;

    public void Customer(String name, int age, char gender) {
        this.name = name;
        this.age = age;
        this.gender = gender;

        this.card = new RentalCard(name);
    }
}
