caffemodel=$1
prototxt=$2
pre=${caffemodel%.*}
echo $pre 
mvNCCompile $2 -w ./$caffemodel -s 12 -o $pre.graph_416 -ec
