# roomba_tasks（地図の上の仕事：ウェイポイント・くまなく掃除）

- `task_manager`（ノード名 `tasks`）：Foxglove でクリックした点（`/clicked_point`）を使って
  - 点を順番に回る（Nav2 の FollowWaypoints）
  - 点で囲んだ範囲を、往復のジグザグでくまなく走る（経路は自分で作り、Nav2 の controller_server の FollowPath に渡す）
- `include/roomba_tasks/coverage.hpp`：くまなく走る経路を作る部分（ROS なしでテストできる）
  - 地図をロボットの半径ぶん太らせる → ロボットからつながっている所だけ → 範囲で切る
  - 平行なレーンを引き、障害物で分かれた「区画」ごとにジグザグで走る。区画の順番は、今いる所から壁を回り込んだ道のりで近い順
  - レーンの向きは、区画とレーンの数が一番少ない向きを自動で選ぶ
- 進めなくなったら（物が置いてあるなど）、少し先の点まで Nav2 の planner で回り道を作って続ける

テスト：`colcon test --packages-select roomba_tasks`、または `g++ -std=c++17 -Iinclude test/test_coverage.cpp && ./a.out`
