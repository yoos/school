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
  * Dynamic vs. static runtime stack -- look at slide 23 for (maybe)
    a higher-level explanation.

# Passing Parameters (05/21)
  * Call-By-Value: pass in values. Local assignments are lost after function
    return.
  * Call-By-Reference: pass in variables only, put pointers on activation
    stack.
  * Call-By-Value-Result, aka copy-in, copy-out. Parameter passed into function
    is modified upon function return. See Parameter Passing slide 15.
  * Call-By-Name: parameters are not evaluated if not used in function, unlike
    CBV, which always evaluates first. Every read access to a parameter
    evaluates it anew. See slides 16 and 17.
  * Call-By-Need: Like Call-By-Name, but expressions are replaced values (i.e.,
    evaluated only once).
  * Table of comparison on slide 20. Classification on slide 21.

# Programming Paradigms (05/21)
  * Model of computing.
  * Imperative: computation is a transformation of states.
  * Functional: computation is a function.
  * Logic: computation is a relation. Unlike a function, a relation can be
    one-to-many.
  * Object-oriented: computation is a manipulation of objects (using methods).

<!--
vim: syntax=markdown
vim: expandtab
-->

