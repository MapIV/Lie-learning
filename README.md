# Lie-learning
## 学習の目的
- リー群・リー代数を習得して活用すると、特に３次元点群のマッチングを行う上でどんないいことがあるか？を知る

- 論文やコードの理解がより進むようになる
  - 例：[IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation](https://www.roboticsproceedings.org/rss11/p06.pdf)のⅡ.A
  - 例：[small_gicp icp_factor.hpp](https://github.com/koide3/small_gicp/blob/master/include/small_gicp/factors/icp_factor.hpp)
  - 例：[gtsam Pose3.cpp](https://github.com/borglab/gtsam/blob/1530c9be3ace8a97452c74e2317ad8f39217f8a7/gtsam/geometry/Pose3.cpp#L1)

## 対象
- 大学の学部１年生で習うレベルの線形代数を履修された方で、ロボティクス分野で用いるリー群・リー代数の基礎を学びたい方

## 章構成と目標
### 1.[リー群編](https://qiita.com/koki2022/private/0310ffb05ac0a39a5203)
$\boldsymbol{R} \in SO(3)$や$\boldsymbol{T} = [\boldsymbol{R} \mid \boldsymbol{t}] \in SE(3)$ について群の定義を踏まえた上で理解しよう

### 2.[リー代数編](https://qiita.com/koki2022/private/bf43a6dbe8208cbf84b1)
リー群に付随するリー代数を理解しよう

### 3.[指数写像と対数写像編](https://qiita.com/koki2022/private/b609c9472ec857ca64f2)
指数写像と対数写像とはなにか理解しよう

### 4.[ICP registration編](https://qiita.com/koki2022/private/532edc226e4579b5b5b4)
なぜリー代数を用いた位置姿勢のパラメータの最適化が便利なのか理解しよう

### 5.随伴表現編
（未定）

### 6.クオータニオン編
（未定）

### 7.不確かさ編
（未定）

## その他
- 定義と実用例を踏まえた説明に注力しています。証明は省略しています。
- ネット上にはすでにとてもわかりやすい記事がたくさんあるため、あまり完璧な解説書としての役割は目指していません。そのため、図はネット記事に上がっているもののリンクを参照して活用させていただいています。
- リー群・リー代数については日々勉強中で、自分としてはそのアウトプットの一環でもあります。間違っているところがあればISSUEを投げていただけると助かります。

## 株式会社マップフォーについて
このシリーズの記事は、私が現在勤務しております株式会社マップフォーでの勉強会向けに作成した資料です。そのため、以下に弊社の紹介をさせていただきます。

株式会社マップフォーは、3次元点群地図作成のための計測機器およびソフトウェアの開発を行っております。ロボティクス分野に留まらず、インフラ、交通など幅広い分野のお客様の課題解決に取り組んでおり、実際に全国各地の現場に出向いて業務を行うなど、非常にやりがいのある事業を展開しております。

現在、私たちと共に働いていただける方を積極的に募集しております。また、学生インターンシップも実施しておりますので、ご興味をお持ちの方は、ぜひkoki.aoki@map4.jpまでお気軽にお問い合わせください。
