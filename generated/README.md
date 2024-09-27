```bash
rm -rf python/*
python -m grpc_tools.protoc -I ../protos/ --python_out=./python/ --pyi_out=./python/ --grpc_python_out=./python/ ../protos/rb/api/*.proto
```

python -m grpc_tools.protoc -I ./
--python_out=./python/
--pyi_out=./python/
--grpc_python_out=./python/
./rb/api/*.proto

