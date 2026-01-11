using System;
using System.Collections.Generic;
using System.Linq;

namespace Hagoromo.MathTools
{
    // スパース行列の要素 (行, 列, 値)
    public struct MatrixElement
    {
        public int Row;
        public int Col;
        public double Val;

        public MatrixElement(int r, int c, double v)
        {
            Row = r;
            Col = c;
            Val = v;
        }
    }

    /// <summary>
    /// 制約式を受け付けるビルダークラス
    /// これが「行番号」を裏で勝手にカウントアップしてくれる
    /// </summary>
    public class ConstraintBuilder
    {
        public List<MatrixElement> J_Elements { get; } = new List<MatrixElement>();
        public List<double> Residuals { get; } = new List<double>();

        // 現在の行番号（自動管理）
        public int CurrentRowIndex { get; private set; } = 0;

        /// <summary>
        /// 現在の行(CurrentRowIndex)の、指定した列(col)に、微分値(val)を足し込む
        /// </summary>
        public void AddDerivative(int col, double val)
        {
            // 値がほぼ0なら無視してスパース性を保つ（任意）
            if (Math.Abs(val) < 1e-12) return;

            J_Elements.Add(new MatrixElement(CurrentRowIndex, col, val));
        }

        /// <summary>
        /// 現在の行の残差(b)を設定（加算）する
        /// </summary>
        public void AddResidual(double val)
        {
            // まだリストが足りてなければ現在の行まで埋める
            while (Residuals.Count <= CurrentRowIndex) Residuals.Add(0.0);

            Residuals[CurrentRowIndex] += val;
        }

        /// <summary>
        /// 現在の行の編集を終了し、次の行へ進む
        /// </summary>
        public void NextRow()
        {
            // もし残差が一度も設定されていなければ、0として埋めておく
            while (Residuals.Count <= CurrentRowIndex) Residuals.Add(0.0);

            CurrentRowIndex++;
        }


        /// <summary>
        /// 新しい行を1つ確保し、その行番号(ID)を返します。
        /// このIDを使えば、後で AddDerivative(row, ...) を使って書き込めます。
        /// </summary>
        public int AllocateRow()
        {
            int assignedIndex = CurrentRowIndex;
            EnsureResidualsCapacity(assignedIndex);

            // カウンタを進める
            CurrentRowIndex++;

            return assignedIndex;
        }

        /// <summary>
        /// 指定した行(rowIndex)・列(col)に微分値を追加します。
        /// </summary>
        public void AddDerivative(int rowIndex, int col, double val)
        {
            // 値がほぼ0なら無視してスパース性を保つ
            if (Math.Abs(val) < 1e-12) return;

            J_Elements.Add(new MatrixElement(rowIndex, col, val));
        }

        /// <summary>
        /// 指定した行(rowIndex)に残差(b)を加算します。
        /// </summary>
        public void AddResidual(int rowIndex, double val)
        {
            EnsureResidualsCapacity(rowIndex);
            Residuals[rowIndex] += val;
        }

        /// <summary>
        /// 新しい制約式を1つ追加する
        /// </summary>
        /// <param name="residual">残差 (bの値)</param>
        /// <param name="gradients">変数ごとの微分値 (col, val) のペア</param>
        public void AddConstraint(double residual, params (int col, double val)[] gradients)
        {
            // bベクトルに追加
            Residuals.Add(residual);

            // J行列に追加（行番号は自動で CurrentRowIndex が使われる）
            foreach (var g in gradients)
            {
                J_Elements.Add(new MatrixElement(CurrentRowIndex, g.col, g.val));
            }

            // 行番号を次に進める
            CurrentRowIndex++;
        }

        private void EnsureResidualsCapacity(int rowIndex)
        {
            // 指定された行番号までリストを0埋めする
            while (Residuals.Count <= rowIndex)
            {
                Residuals.Add(0.0);
            }
        }

        // リセット用
        public void Clear()
        {
            J_Elements.Clear();
            Residuals.Clear();
            CurrentRowIndex = 0;
        }
    }

    /// <summary>
    /// 最小二乗法の各項（制約条件）の基底クラス
    /// L-BFGSのEnergyTermに相当
    /// </summary>
    public abstract class LeastSquaresTerm
    {
        public double Weight { get; set; }
        protected LeastSquaresTerm(double weight) { Weight = weight; }

        // 行数とかオフセットとか気にせず、builderに足すだけでいい
        public abstract void AddToSystem(double[] x, ConstraintBuilder builder);

        public virtual bool IsFeasible(double[] x)
        {
            return true;
        }
    }


    

    public class GaussNewtonOptimizer
    {
        public int N { get; private set; } // 変数の数
        public double[] X { get; private set; } // 状態ベクトル

        private List<LeastSquaresTerm> terms = new List<LeastSquaresTerm>();

        private ConstraintBuilder builder = new ConstraintBuilder();

        public GaussNewtonOptimizer(int variableCount)
        {
            N = variableCount;
            X = new double[N];
        }

        // 項を追加する
        public void AddTerm(LeastSquaresTerm term)
        {
            terms.Add(term);
        }

        // 1ステップ進める（ガウス・ニュートン法）
        // outerIter: 行列を作り直す回数
        // innerIter: CGLSの内部反復回数
        public void Step(int outerIter = 3, int innerIter = 100)
        {
            for (int k = 0; k < outerIter; k++)
            {
                SolveOneStep(innerIter);
            }
        }

        private void SolveOneStep(int maxCGLSIter)
        {
            // 1. ビルダーをリセット
            builder.Clear();

            // 2. 全ての項に「式を入れてくれ」と依頼
            foreach (var term in terms)
            {
                term.AddToSystem(X, builder);
            }

            // 3. ビルダーが集めたデータを取り出す
            int M = builder.CurrentRowIndex; // 最終的に何行になったかがここに入っている
            List<MatrixElement> J_elements = builder.J_Elements;
            double[] b = builder.Residuals.ToArray();


            // CGLS用の行列積関数の作成
            // J * v
            Func<double[], double[]> multiplyJ = (v) =>
            {
                double[] res = new double[M];
                // 並列化可能 (Parallel.ForEach)
                foreach (var el in J_elements)
                {
                    res[el.Row] += el.Val * v[el.Col];
                }
                return res;
            };

            // J^T * v
            Func<double[], double[]> multiplyJT = (v) =>
            {
                double[] res = new double[N];
                foreach (var el in J_elements)
                {
                    res[el.Col] += el.Val * v[el.Row];
                }
                return res;
            };

            // 4. CGLSで Delta X を求める
            double[] deltaX = new double[N];
            CGLS_Solver.Solve(M, N, multiplyJ, multiplyJT, b, deltaX, maxCGLSIter);

            // ★ --- ラインサーチ (Line Search) 開始 --- ★
            double alpha = 1.0;      // 初期ステップ幅
            double minAlpha = 1e-4;  // 最小ステップ幅（これ以上小さくなると諦める）
            double[] nextX = new double[N];
            bool feasible = false;

            while (alpha >= minAlpha)
            {
                // 次の候補点を計算: X_new = X + alpha * deltaX
                for (int i = 0; i < N; i++)
                {
                    nextX[i] = X[i] + alpha * deltaX[i];
                }

                // 全ての項に対して制約チェック
                feasible = true;
                foreach (var term in terms)
                {
                    if (!term.IsFeasible(nextX))
                    {
                        feasible = false;
                        break;
                    }
                }

                if (feasible) break; // 全てOKならループを抜ける

                alpha *= 0.5; // 制約に違反していたらステップ幅を半分にする
            }

            // 更新（制約を満たせなかった場合はalpha=0に近い状態で微小移動、または移動しない）
            if (feasible)
            {
                Array.Copy(nextX, X, N);
            }
        }
    }

    
    public static class CGLS_Solver
    {
        /// <summary>
        /// J^T J x = J^T b を解く (最小二乗法)
        /// </summary>
        public static void Solve(
            int m, int n,
            Func<double[], double[]> multiplyJ,
            Func<double[], double[]> multiplyJT,
            double[] b,
            double[] x, // 結果はここに書き込まれる(初期値兼出力)
            int maxIter = 100,
            double tolerance = 1e-6)
        {
            // r = b - Jx (初期値x=0なら r=b)
            // ここでは初期x=0と仮定して r=b からスタートでも良いが、
            // 汎用性のために真面目に計算する
            double[] Jx = multiplyJ(x);
            double[] r = new double[m];
            for (int i = 0; i < m; i++) r[i] = b[i] - Jx[i];

            // p = J^T * r
            double[] p = multiplyJT(r);

            // s = p (共役方向)
            double[] s = (double[])p.Clone();

            double normSq_p = Dot(p, p);
            if (normSq_p < tolerance * tolerance) return;

            for (int k = 0; k < maxIter; k++)
            {
                // q = J * s
                double[] q = multiplyJ(s);

                double normSq_q = Dot(q, q);
                // ゼロ除算回避
                if (normSq_q < 1e-20) break;

                // alpha = ||p||^2 / ||q||^2
                double alpha = normSq_p / normSq_q;

                // x = x + alpha * s
                for (int i = 0; i < n; i++) x[i] += alpha * s[i];

                // r = r - alpha * q
                for (int i = 0; i < m; i++) r[i] -= alpha * q[i];

                // p_new = J^T * r
                double[] p_new = multiplyJT(r);
                double normSq_p_new = Dot(p_new, p_new);

                if (normSq_p_new < tolerance * tolerance) break;

                // beta = ||p_new||^2 / ||p||^2
                double beta = normSq_p_new / normSq_p;

                // s = p_new + beta * s
                for (int i = 0; i < n; i++) s[i] = p_new[i] + beta * s[i];

                p = p_new;
                normSq_p = normSq_p_new;
            }
        }

        private static double Dot(double[] a, double[] b)
        {
            double sum = 0;
            for (int i = 0; i < a.Length; i++) sum += a[i] * b[i];
            return sum;
        }
    }
}
