# CS381

Class website: http://classes.engr.oregonstate.edu/~erwig/cs381

## 04/02
  * Syntax, semantics
  * Computation: systematic transformation of representation
  * What is a program? A programming language?
  * Metalanguages allow us to describe other languages. Haskell can help us
    understand functional programming and be used as a metalanguage
  * Doing vs. being; operational vs. declarative views
  * Higher-order functions (e.g., fold, map)

## 04/04
  * Review data contruction.

## 04/09
  * Quiz: Note that
        head :: [a] -> a
        tail :: [a] -> [a]

# 04/16
  * Abstract vs concrete syntaxes.
  * Pretty printer (and parser, which is pretty print in reverse) using
    abstract syntax?
  * When writing grammars, consider features of the metalanguage and how they
    can help me.
  * Remember that each case of a data type (that is a nonterminal) must have
    a constructor!

# 04/18
  * Syntax vs. semantics.

# 04/23
  * Data with one constructor can also be represented as a type.
  * Semantic functions and domains
  * Monads allow for nicer (modular?) error handling.
  * Know how the Maybe type works.
  * Try extending ExprErr.hs with error handlers.

# 04/30
  * Type safety and static vs dynamic typing
  * Food for thought: what is the type of f x = f (x+1) * 2? (Note that it does
    not terminate but "returns" an Int.) A type checker is not a termination
    checker.
  * A type checker is like a semantic function.

# 05/02
  * Think about:
        undefined :: a
        undefined = undefined

# 05/16
  * Access links useful for static scoping implementation.
  * Closure: function expression + environment. This allows the function to
    remember local variables after termination.
  * Dynamic vs. static runtime stack -- look at slide 23 for (maybe) a higher-level explanation.


<!--
vim: syntax=markdown
vim: expandtab
-->

