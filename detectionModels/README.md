Two models are uploaded:

1. The yolo11m model trained in ultralytics hub, and downloaded in pyTorch format 

2. Model quantized for Axelera metis via Degirum-hub online compiler


NOTE: The first model in pyTorch format was what was fed into the degirum online compmiler to create the quantized model. The quantized model can be directly used on Axelera hardware using Degirum's pysdk.

update: The original yolo11m model could not be uploaded due to file size constraints in github. Get in touch with me if you need the original model and I will arrange to share with you via an alternate way. The output of the Deigirum compiler is uploaded and this is what you need to run it on the Axelera hardware via degirum's pysdk.
