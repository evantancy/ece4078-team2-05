# Milestone 3 (M3)
## Known issues
Error: ``AttributeError: module 'yaml' has no attribute 'FullLoader'`` this occurs if your PyYAML version is lower than 5.1
Check your PyYAML version by running the following from CLI
```bash
python3
import yaml
yaml.__version__
```