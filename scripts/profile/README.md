
https://docs.mongodb.com/manual/reference/method/js-collection/
https://docs.mongodb.com/manual/reference/command/collStats/#dbcmd.collStats


collStats.size : Tamaño de documentos. SIN CONTAR ÍNDICES

    The total size in memory of all records in a collection. This value does not include the record header, which is 16 bytes per record, but does include the record’s padding. Additionally size does not include the size of any indexes associated with the collection, which the totalIndexSize field reports.

    The scale argument affects this value.


collStats.storageSize : Tamaño reservado para almacenamiento; SIN CONTAR INDICES

    The total amount of storage allocated to this collection for document storage. The scale argument affects this value.

    storageSize does not include index size. See totalIndexSize for index sizing.

    For MMAPv1, storageSize will not decrease as you remove or shrink documents.




https://docs.mongodb.com/manual/reference/command/dbStats/

dbStats.dataSize : Tamaño total de los documentos en la DB.

    The total size of the uncompressed data held in this database. The dataSize decreases when you remove documents.

    For databases using the MMAPv1 storage engine, dataSize includes preallocated space and the padding factor. The dataSize does not decrease when documents shrink.

    For databases using the WiredTiger storage engine, dataSize may be larger than storageSize if compression is enabled. The dataSize decreases when documents shrink.

dbStats.storageSize : Tamaño reservado para almacenamiento

    The total amount of space allocated to collections in this database for document storage. The storageSize does not decrease as you remove or shrink documents. This value may be smaller than dataSize for databases using the WiredTiger storage engine with compression enabled.

dbStats.fileSize : Tamaño total de la base de datos en disco.

    The total size of the data files that hold the database. This value includes preallocated space and the padding factor. The value of fileSize only reflects the size of the data files for the database and not the namespace file.

    Only present when using the mmapv1 storage engine.
