```mermaid
graph TD;
  A[Trigger Test: trigger_run.py AWS CodeBuild / Manual \n Executed in AWS / Lab PC] -->|HTTPS request| B[GitHub Action Initiated \n Executed in GitHub Actions (Cloud)];

  B --> C[Lab PC GitHub Agent \n Executed in Lab PC (On-Prem)];

  C --> D[get_apk.py - Fetch APK from Firebase \n Executed in Lab PC];
  C --> E[check_connected_phones.py - Verify Devices \n Executed in Lab PC];
  C --> F[squarepeg_control.py - Setup Satellite Simulator \n Executed in Lab PC];

  D --> G[subprocess_manager.py - Start Tests \n Executed in Lab PC];
  E --> G;
  F --> G;

  G --> H[Logs Stored Locally \n Executed in Lab PC];
  G --> I[Logs Uploaded to GitHub \n Stored in GitHub Storage (Cloud)];
```