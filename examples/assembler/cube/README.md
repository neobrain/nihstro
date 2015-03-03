cube example
============

Simple port of ctrulib's gpu example to nihstro shaders. The C program code is mostly unchanged from the original, however the example shader in the data subdirectory should give you a good idea of the basic nihcode shader syntax.

Before trying to compile, make sure your NIHSTRO environment variable points to the directory nihstro-assemble resides in. Additionally, ctrulib in revision 1f52ac344d or similar is required, plus some patches to implement proper uniform setters.
