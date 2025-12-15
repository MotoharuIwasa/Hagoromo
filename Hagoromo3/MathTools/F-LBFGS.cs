using Hagoromo.GeometryTools;
using MathNet.Numerics.Optimization;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hagoromo.MathTools
{
    //これはCutMeshなどでなくてもどの問題でも使える。
    public static class LBFGS_Solver
    {
        // 履歴データの構造体
        private class HistoryItem
        {
            public double[] s;   // x の変化量 (delta X)
            public double[] y;   // g の変化量 (delta Gradient)
            public double rho;   // 1 / (y^T * s)
        }

        /// <summary>
        /// L-BFGS法による最適化実行
        /// </summary>
        /// <param name="n">変数の数 (頂点数 * 2 or 3)</param>
        /// <param name="x">初期位置 (兼 結果格納用)</param>
        /// <param name="calcEnergy">現在の x における全エネルギー値を返す関数</param>
        /// <param name="calcGradient">現在の x における全勾配ベクトル(g)を返す関数</param>
        /// <param name="isValidGeometry">幾何学的制約(裏返り等)をチェックする関数 (LineSearch用)</param>
        /// <param name="m">履歴サイズ (通常 5〜20)</param>
        public static void Solve(
            IOptimizationProblem problem, // ← 変更: 関数ポインタの代わりに問題オブジェクトを受け取る
            double[] x,
            int maxIter = 100,
            int m = 10,
            double tolerance = 1e-6)
        {
            int n = problem.Dimension;

            // 勾配配列の確保
            double[] g = new double[n];

            // 履歴リスト
            LinkedList<HistoryItem> history = new LinkedList<HistoryItem>();

            // 【変更点1】 初期化: エネルギーと勾配を一括計算！
            // 別々に呼ぶと2回計算することになるが、これなら1回で済む
            double energy = problem.CalculateEnergyAndGradient(x, g);

            // 勾配の大きさチェック
            if (Dot(g, g) < tolerance * tolerance) return;

            // メインループ
            for (int k = 0; k < maxIter; k++)
            {
                // 1. 探索方向 d
                double[] d = ComputeDirection(g, history, n);

                // 2. ラインサーチ
                // ※ラインサーチ中はエネルギーだけ計算できればいいので CalculateEnergy を使う
                double alpha = LineSearch(n, x, d, g, energy, problem.CalculateEnergy, problem.IsValid);

                if (alpha == 0.0) break;

                // 3. 更新前の情報を保持
                double[] x_old = (double[])x.Clone();
                double[] g_old = (double[])g.Clone();

                // 4. 位置更新
                for (int i = 0; i < n; i++) x[i] += alpha * d[i];

                // 5. 新しい状態の計算
                // 【変更点2】 ここも一括計算！ 高速化の肝
                double newEnergy = problem.CalculateEnergyAndGradient(x, g); // g はここで上書き更新される

                // Note: newG という新しい配列を作らなくて済むのでメモリ効率も良い

                // 6. 履歴の更新
                double[] s = new double[n];
                double[] y = new double[n];
                for (int i = 0; i < n; i++)
                {
                    s[i] = x[i] - x_old[i];
                    y[i] = g[i] - g_old[i]; // g はすでに新しい値になっている
                }

                double ys = Dot(y, s);
                if (ys > 1e-10)
                {
                    var item = new HistoryItem { s = s, y = y, rho = 1.0 / ys };
                    history.AddLast(item);
                    if (history.Count > m) history.RemoveFirst();
                }

                // 次のループへの準備
                energy = newEnergy;
                // g は既に更新済みなので代入不要

                if (Dot(g, g) < tolerance * tolerance) break;
            }
        }

        // --- Two-Loop Recursion (L-BFGSの核心) ---
        // 過去の s, y を使って、擬似的に H^(-1) * g を計算する
        private static double[] ComputeDirection(double[] g, LinkedList<HistoryItem> history, int n)
        {
            // q = g (コピー)
            double[] q = (double[])g.Clone();

            // alphas配列 (履歴の数だけ必要)
            double[] alphas = new double[history.Count];

            // [Backward Loop] 最新 -> 過去
            int idx = history.Count - 1;
            foreach (var item in history.Reverse())
            {
                // alpha = rho * (s^T * q)
                double alpha = item.rho * Dot(item.s, q);
                alphas[idx] = alpha;

                // q = q - alpha * y
                for (int i = 0; i < n; i++) q[i] -= alpha * item.y[i];
                idx--;
            }

            // [Scaling] 初期のヘッセ行列 H0 の推定
            // r = H0 * q
            double[] r = (double[])q.Clone();
            if (history.Count > 0)
            {
                var last = history.Last.Value;
                // gamma = (s^T y) / (y^T y)
                double gamma = Dot(last.s, last.y) / Dot(last.y, last.y);
                for (int i = 0; i < n; i++) r[i] *= gamma;
            }

            // [Forward Loop] 過去 -> 最新
            idx = 0;
            foreach (var item in history)
            {
                double alpha = alphas[idx];
                // beta = rho * (y^T * r)
                double beta = item.rho * Dot(item.y, r);

                // r = r + s * (alpha - beta)
                double coeff = alpha - beta;
                for (int i = 0; i < n; i++) r[i] += coeff * item.s[i];
                idx++;
            }

            // 方向 d = -r (勾配と逆向き)
            for (int i = 0; i < n; i++) r[i] = -r[i];

            return r; // これが d
        }

        // --- Line Search (Backtracking / Armijo) ---
        private static double LineSearch(
            int n,
            double[] x,
            double[] d,
            double[] g,
            double currentEnergy,
            Func<double[], double> calcEnergy,
            Func<double[], bool> isValidGeometry)
        {
            double alpha = 1.0;
            double c1 = 1e-4;
            double g_dot_d = Dot(g, d); // 勾配方向への射影 (必ず負になるはず)

            // もし d が上昇方向ならリセット（数値誤差対策）
            if (g_dot_d > 0) return 0.0;

            double[] x_try = new double[n];

            // 最大20回くらいトライ
            for (int i = 0; i < 20; i++)
            {
                // x_try = x + alpha * d
                for (int k = 0; k < n; k++) x_try[k] = x[k] + alpha * d[k];

                // 1. 幾何学的チェック (裏返り検知)
                // バリア関数を使う場合は、ここが重要。裏返ったらエネルギー計算すら危険なので即NG。
                if (!isValidGeometry(x_try))
                {
                    alpha *= 0.5;
                    continue;
                }

                // 2. エネルギー減少チェック (Armijo条件)
                // E_new <= E_old + c1 * alpha * (g^T d)
                double newEnergy = calcEnergy(x_try);

                if (newEnergy <= currentEnergy + c1 * alpha * g_dot_d)
                {
                    return alpha; // 採用
                }

                // ダメなら歩幅を半分にする
                alpha *= 0.5;
            }

            return 0.0; // 失敗
        }

        // ヘルパー: 内積
        private static double Dot(double[] a, double[] b)
        {
            double sum = 0.0;
            for (int i = 0; i < a.Length; i++) sum += a[i] * b[i];
            return sum;
        }
    }

    public abstract class EnergyTerm
    {
        public double Weight { get; set; }

        protected EnergyTerm(double weight) { Weight = weight; }

        // エネルギー値を返す
        public abstract double GetEnergy(double[] x);

        // 勾配ベクトル g に自身の寄与を足し込む (AddGradient)
        // ※新しい配列を作らず、渡された g 配列に += することで高速化
        public abstract void AddGradient(double[] x, double[] g);

        //エネルギー関数の計算と勾配の計算で同じ計算を2回することがないようにまとめて計算するためのツール。使っても使わなくてもOK
        public virtual double Evaluate(double[] x, double[] g)
        {
            // デフォルト実装: 今まで通り別々に呼ぶ（オーバーライドしなければこれを使えば良い）
            if (g != null)
            {
                AddGradient(x, g);
            }
            return GetEnergy(x);
        }
    }

    public interface IOptimizationProblem
    {
        int Dimension { get; }

        // ラインサーチ用（エネルギーだけ欲しいとき）
        double CalculateEnergy(double[] x);

        // メインループ用（エネルギーと勾配を同時に計算して高速化）
        double CalculateEnergyAndGradient(double[] x, double[] g);

        // 制約チェック用
        bool IsValid(double[] x);
    }

    // 不等式制約（Line Searchでの「壁」判定用）
    // 満たしていれば true, 破っていれば false
    public interface IInequalityConstraint
    {
        bool IsSatisfied(double[] x);
    }

    public class SimulationSystem : IOptimizationProblem
    {
        // 変数
        public int N { get; private set; } // 全要素数 (頂点数 * 2)
        public double[] X { get; private set; } // 状態ベクトル

        // 登録されたエネルギー項と制約のリスト
        private List<EnergyTerm> energyTerms = new List<EnergyTerm>();
        private List<IInequalityConstraint> constraints = new List<IInequalityConstraint>();

        public SimulationSystem(int variableCount)
        {
            N = variableCount;
            X = new double[N];
        }


        // --- インターフェースの実装 ---

        public int Dimension => N;

        // IsGeometryValid を IsValid として公開
        public bool IsValid(double[] x) => IsGeometryValid(x);

        // CalculateTotalEnergy を CalculateEnergy として公開
        // ※高速化のため Evaluate(x, null) を呼ぶように変更推奨
        public double CalculateEnergy(double[] x)
        {
            double totalE = 0.0;
            foreach (var term in energyTerms)
            {
                // g=null なのでエネルギー計算だけが行われる
                totalE += term.Evaluate(x, null);
            }
            return totalE;
        }

        // すでに実装済みのこれをインターフェース経由で呼べるようにする
        public double CalculateEnergyAndGradient(double[] x, double[] g)
        {
            Array.Clear(g, 0, g.Length);
            double totalEnergy = 0.0;
            foreach (var term in energyTerms)
            {
                totalEnergy += term.Evaluate(x, g);
            }
            return totalEnergy;
        }



        // --- 追加メソッド ---
        public void AddEnergy(EnergyTerm term) => energyTerms.Add(term);
        public void AddConstraint(IInequalityConstraint c) => constraints.Add(c);

        // --- L-BFGSから呼ばれる関数 ---

        // 1. 全エネルギー計算
        public double CalculateTotalEnergy(double[] currentX)
        {
            double totalE = 0.0;
            foreach (var term in energyTerms)
            {
                totalE += term.GetEnergy(currentX);
            }
            return totalE;
        }

        // 2. 全勾配計算
        public double[] CalculateTotalGradient(double[] currentX)
        {
            double[] g = new double[N]; // 0初期化

            // 全ての項の勾配を足し合わせる
            foreach (var term in energyTerms)
            {
                term.AddGradient(currentX, g);
            }
            return g;
        }

        // 3. 幾何学的妥当性チェック
        public bool IsGeometryValid(double[] currentX)
        {
            foreach (var c in constraints)
            {
                if (!c.IsSatisfied(currentX)) return false;
            }
            return true;
        }

        // --- 実行 ---
        public void Step(int iteration = 100)
        {
            // ここでL-BFGSソルバーを呼ぶ
            // 前回の回答の Solver.Solve を利用するイメージ
            LBFGS_Solver.Solve(
                this,
                X,
                maxIter: iteration,
                m: 10
            );
        }
    }
    

}
