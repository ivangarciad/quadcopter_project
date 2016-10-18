ctags -f tags.cpp `find . -name "*.cpp" -o -name "*.h"`
ctags -f tags.ino --langmap=c++:.ino `find . -name "*.ino"`
cat tags.cpp tags.ino > tags
sort tags -o tags
rm -f tags.* 
