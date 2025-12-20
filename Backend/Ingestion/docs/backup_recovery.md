# Backup and Recovery Procedures for Vector Database

This document outlines the backup and recovery procedures for the Qdrant vector database used in the book ingestion system.

## Overview

The Book Ingestion System stores vector embeddings in Qdrant, which requires regular backup procedures to ensure data availability and system reliability. This document provides guidelines for backing up and recovering vector database collections.

## Backup Strategy

### 1. Full Backups

Full backups of Qdrant collections should be performed regularly:

- **Frequency**: Daily for production systems
- **Retention**: Keep 7 daily backups, 4 weekly backups, 12 monthly backups
- **Storage**: Store backups in secure, geographically distributed locations

### 2. Incremental Backups

For large collections, consider incremental backups:
- **Frequency**: Every 6 hours
- **Retention**: Keep 7 days of incremental backups
- **Storage**: Use the same secure storage as full backups

## Backup Procedures

### Manual Backup

```bash
# Using Qdrant HTTP API to create a snapshot
curl -X POST "http://your-qdrant-url/collections/{collection_name}/snapshots" \
  -H "api-key: your-api-key"
```

### Automated Backup Script

Create a backup script for automated execution:

```bash
#!/bin/bash

# backup_qdrant.sh
QDRANT_URL="your-qdrant-url"
API_KEY="your-api-key"
COLLECTION_NAME="book_embeddings"
BACKUP_DIR="/path/to/backup/location"
DATE=$(date +%Y%m%d_%H%M%S)

# Create snapshot
SNAPSHOT_RESPONSE=$(curl -s -X POST \
  "$QDRANT_URL/collections/$COLLECTION_NAME/snapshots" \
  -H "api-key: $API_KEY")

SNAPSHOT_NAME=$(echo $SNAPSHOT_RESPONSE | jq -r '.result.name')

# Download snapshot (if supported by your Qdrant setup)
# This part may vary depending on your Qdrant deployment
```

### Using Qdrant Cloud Features

If using Qdrant Cloud, utilize built-in backup features:

1. **Automatic Snapshots**: Enable automatic daily snapshots in the Qdrant Cloud console
2. **Cross-Region Replication**: Set up replication to another region for disaster recovery
3. **Retention Policies**: Configure retention policies to automatically manage backup lifecycle

## Recovery Procedures

### 1. Collection Recovery

To restore a specific collection from a snapshot:

```bash
# Restore from snapshot using Qdrant API
curl -X PUT "http://your-qdrant-url/collections/{collection_name}/snapshots/{snapshot_name}" \
  -H "api-key: your-api-key" \
  -H "Content-Type: application/json" \
  -d '{
    "snapshot": "{snapshot_name}"
  }'
```

### 2. Complete System Recovery

In case of complete system failure:

1. **Restore Qdrant Instance**: Set up a new Qdrant instance
2. **Restore Collections**: Restore each collection from the latest backup
3. **Verify Data**: Check that all vectors and metadata are correctly restored
4. **Update Configuration**: Update any system configuration to point to the new instance
5. **Test System**: Run end-to-end tests to ensure the system functions correctly

### 3. Partial Recovery

To recover specific data:

1. **Identify Missing Data**: Determine which documents/chunks need recovery
2. **Re-process Documents**: Run the ingestion pipeline on the source documents
3. **Verify Integrity**: Check that the recovered data matches the original

## Backup Verification

### 1. Automated Verification

Implement automated checks to verify backup integrity:

```python
# verify_backup.py
import requests
from datetime import datetime, timedelta

def verify_latest_backup(collection_name, qdrant_url, api_key):
    """Verify the latest backup is valid."""
    response = requests.get(
        f"{qdrant_url}/collections/{collection_name}/snapshots",
        headers={"api-key": api_key}
    )

    snapshots = response.json()["result"]["snapshots"]
    if not snapshots:
        raise Exception("No snapshots found")

    latest_snapshot = max(snapshots, key=lambda x: x["creation_time"])
    age = datetime.now() - datetime.fromisoformat(latest_snapshot["creation_time"][:-1])

    if age > timedelta(hours=25):  # More than 24 hours + 1 hour buffer
        raise Exception(f"Latest backup is {age} old, exceeds 24-hour requirement")

    print(f"Latest backup is {age} old and appears valid")
    return True
```

### 2. Manual Verification

Regular manual checks should include:

- **Count Verification**: Compare vector counts before and after backup
- **Sample Testing**: Query random samples to ensure content is preserved
- **Metadata Integrity**: Verify that all metadata fields are correctly restored

## Disaster Recovery Plan

### 1. RTO and RPO Targets

- **Recovery Time Objective (RTO)**: 2 hours for production systems
- **Recovery Point Objective (RPO)**: 24 hours for data loss

### 2. Recovery Team Roles

- **Database Administrator**: Responsible for Qdrant recovery operations
- **System Administrator**: Manages infrastructure recovery
- **Developer**: Validates application functionality after recovery

### 3. Recovery Steps

1. **Assessment** (0-15 minutes)
   - Determine scope of failure
   - Identify required backup sets

2. **Infrastructure Setup** (15-45 minutes)
   - Set up new Qdrant instance if needed
   - Configure network and security settings

3. **Data Restoration** (45-90 minutes)
   - Restore collections from backups
   - Verify data integrity

4. **System Validation** (30-60 minutes)
   - Test search functionality
   - Validate application integration

5. **Cutover** (15-30 minutes)
   - Update application configuration
   - Resume normal operations

## Security Considerations

### 1. Backup Encryption

- Encrypt backups at rest using strong encryption algorithms
- Manage encryption keys separately from backup data
- Use key rotation policies for long-term security

### 2. Access Control

- Limit access to backup storage to authorized personnel only
- Implement multi-factor authentication for backup systems
- Audit all backup and recovery operations

### 3. Compliance

- Ensure backup procedures comply with data protection regulations
- Maintain proper data retention policies
- Document all backup and recovery activities

## Monitoring and Alerts

### Backup Success Monitoring

Set up monitoring to alert on:

- Backup job failures
- Backup size anomalies
- Backup completion time increases
- Storage capacity issues

### Recovery Readiness

Regularly test recovery procedures:

- **Quarterly**: Full disaster recovery test
- **Monthly**: Partial recovery test
- **Weekly**: Backup verification checks

## Documentation and Training

### 1. Procedure Documentation

Maintain up-to-date documentation for:

- Backup procedures
- Recovery procedures
- Contact information for recovery team
- System dependencies and configurations

### 2. Staff Training

Regular training should cover:

- Backup and recovery procedures
- Emergency response protocols
- Tool usage and troubleshooting
- Security and compliance requirements

## Appendix: Backup Scripts

### Python Backup Script

```python
import qdrant_client
import os
from datetime import datetime
import logging

def create_backup(collection_name: str, backup_path: str):
    """Create a backup of a Qdrant collection."""
    client = qdrant_client.QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    try:
        # Create snapshot
        snapshot_info = client.create_snapshot(collection_name=collection_name)
        logging.info(f"Created snapshot: {snapshot_info.name}")

        # Copy snapshot to backup location (implementation depends on your setup)
        # This would typically involve copying files from Qdrant's snapshot directory
        # to your backup storage location

    except Exception as e:
        logging.error(f"Backup failed: {e}")
        raise e

if __name__ == "__main__":
    COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")
    BACKUP_PATH = os.getenv("BACKUP_PATH", "/backups")

    create_backup(COLLECTION_NAME, BACKUP_PATH)
```

This backup and recovery procedure should be reviewed and updated regularly to ensure it remains effective as the system evolves.